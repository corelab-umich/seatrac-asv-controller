module SyntheticData

using Interpolations

import ..STGPKF

export EnvDataSpatial
export EnvDataST

struct EnvDataSpatial{V, M, F}
    X::V # X-axis
    Y::V # Y-axis
    W::M # Data
    itp_w::F # interpolation of the data
end

struct EnvDataST{V, M, F}
    X::V # X-axis
    Y::V # Y-axis
    T::V # Temporal scale
    W::M # Data
    itp_w::F # interpolation of the data
end

function EnvDataSpatial(X, Y, W)
    try 
        itp_w = linear_interpolation((X,Y), W)
        return EnvDataSpatial(X, Y, W, itp_w)
      catch
        return EnvDataSpatial(X, Y, W, nothing)    
      end
end

function EnvDataST(X, Y, T, W)
    try 
        itp_w = linear_interpolation((X,Y,T), W)
        return EnvDataST(X, Y, T, W, itp_w)
      catch
        return EnvDataST(X, Y, T, W, nothing)    
      end
end


function (eds::EnvDataSpatial)(x, y)
    return eds.itp_w(x, y)
end

function (edst::EnvDataST)(x, y, t)
    return edst.itp_w(x, y, t)
end

using AbstractGPs, KernelFunctions, Random, StaticArrays, LinearAlgebra

# Generate spatial Matern12 synthetic data
# TODO: Currently defaults to ZeroMean Mean Function, overload to include other mean functions
function matern12_spatial(dim, xstart, xstop, ystart, ystop, σ_sq, λₓ)
    # Create the spatial kernel
    spatial_kernel = σ_sq * Matern12Kernel() ∘ ScaleTransform(λₓ);
    
    # Define the mean function
    mean_function = AbstractGPs.ZeroMean();
    
    # Create the Guassian process with the spatial kernel
    gp = AbstractGPs.GP(mean_function, spatial_kernel);

    # Define the spatial grid
    x = range(xstart, stop=xstop, length=dim);
    y = range(ystart, stop=ystop, length=dim);
    spatial_points = vec([SVector(xi, yi) for xi in x, yi in y])

    # Generate synthetic data by sampling from the GP
    y_samp = rand(gp(spatial_points));

    # # If you want reproducible data
    # rng = MersenneTwiser(123);
    # y = rand(rng, gp(spatial_points));

    # Reshape the data into an array
    W = reshape(y_samp, dim, dim);

    X = collect(x)
    Y = collect(y)   
    return EnvDataSpatial(X, Y, W)
end

# Generate spatiotemporal Matern12 synthetic data
# TODO: Currently defaults to ZeroMean Mean Function, overload to include other mean functions
function matern12_spatiotemporal(sdim, tdim, xstart, xstop, ystart, ystop, tstart, tstop, σ_sq, λₓ, λₜ)
    # Create the kernels
    spatial_kernel = Matern12Kernel() ∘ ScaleTransform(λₓ);
    temporal_kernel = Matern12Kernel() ∘ ScaleTransform(λₜ);
    
    # Create spatiotemporal kernel
    spatiotemporal_kernel = σ_sq * spatial_kernel * temporal_kernel;

    # Define the mean function
    mean_function = AbstractGPs.ZeroMean();
    
    # Create the Guassian process with the spatial kernel
    gp = AbstractGPs.GP(mean_function, spatiotemporal_kernel);

    # Define the spatial grid
    x = range(xstart, stop=xstop, length=sdim);
    y = range(ystart, stop=ystop, length=sdim);
    spatial_points = vec([SVector(xi, yi) for xi in x, yi in y]);

    # Define temporal points
    temporal_points = range(tstart, stop=tstop, length=tdim) * 60.0;

    # Create a grid of spatiotemporal points
    spatiotemporal_points = vec([SVector(spatial[1], spatial[2], t) for spatial in spatial_points, t in temporal_points]);

    # Generate synthetic data by sampling from the GP
    y_samp = rand(gp(spatiotemporal_points));

    # # If you want reproducible data
    # rng = MersenneTwister(123);
    # y = rand(rng, gp(spatiotemporal_points))

    # Reshape the data into a 3D array (spatial_x, spatial_y, temporal)
    W = reshape(y_samp, sdim, sdim, tdim);

    # return W
    
    X = collect(x)
    Y = collect(y)   
    T = collect(temporal_points)

    return EnvDataST(X, Y, T, W)
end

# Generate spatiotemporal Matern12 synthetic data
# TODO: Currently defaults to ZeroMean Mean Function, overload to include other mean functions
function synth_stgpkf(sdim, tdim, xstart, xstop, ystart, ystop, ts, Δt, σ, λₛ, λₜ)
   # Define covariance functions
    kₛ(𝐬₁, 𝐬₂) = exp(-λₛ * norm(𝐬₁ - 𝐬₂));
    kₜ(t₁, t₂) = σ^2 * exp(-λₜ * (t₁ - t₂));
    k(𝐬₁, t₁, 𝐬₂, t₂) = kₛ(𝐬₁, 𝐬₂) * kₜ(t₁, t₂);

    # Establish Jordan Lake Domain
    xvals = range(xstart, stop=xstop, length=sdim)
    yvals = range(ystart, stop=ystop, length=sdim)
    domain = Vector{Vector{Float64}}()
    points = [[x, y] for y in yvals for x in xvals]
    for p in points
        push!(domain, p)
    end
    M = length(domain);

    𝐊ₘ = reshape([kₛ(domain[j], domain[i]) for i = 1:M for j = 1:M], (M, M));

    𝐊ₘʰᵃˡᶠ = cholesky(𝐊ₘ).L;
    # 𝐊ₘʰᵃˡᶠ = sqrt(𝐊ₘ)

    F = -λₜ;
    G = 1;
    H = σ * sqrt(2 * λₜ);
    Σ₀ = 1 / (2 * λₜ);
    𝐇ᵇᵃʳ = Diagonal(H * ones(M));

    # Simulate env_data for all i ∈ {1, ..., T / Δt}
    env_data = zeros(M, length(ts))

    Random.seed!(3) # Set seed for reproducibility
    𝐯ᵢ = sqrt(Σ₀) * randn(M)
    for i = 1:length(ts)
        𝐰ᵢᵗⁱˡᵈᵉ = sqrt(1 / (2 * λₜ) * (1 - exp(-2 * λₜ * Δt))) * randn(M)
        𝐯ᵢ = exp(-λₜ * Δt) * 𝐯ᵢ + 𝐰ᵢᵗⁱˡᵈᵉ
        𝐳ᵢ = H * 𝐯ᵢ
        env_data[:, i] = 𝐊ₘʰᵃˡᶠ * 𝐳ᵢ
    end

    # Reshape the data into a 3D array (spatial_x, spatial_y, temporal)
    W = reshape(env_data, sdim, sdim, tdim);

    # return W
    
    X = xvals
    Y = yvals   
    T = range(ts[1], stop=ts[end], length=length(ts))

    return EnvDataST(X, Y, T, W)
end

# Generate spatiotemporal Matern12 synthetic data
# TODO: Currently defaults to ZeroMean Mean Function, overload to include other mean functions
function synth_stgpkf_predef(xmesh, ymesh, ts, Δt, σ, λₛ, λₜ)
    # Define covariance functions
     kₛ(𝐬₁, 𝐬₂) = exp(-λₛ * norm(𝐬₁ - 𝐬₂));
     kₜ(t₁, t₂) = σ^2 * exp(-λₜ * (t₁ - t₂));
     k(𝐬₁, t₁, 𝐬₂, t₂) = kₛ(𝐬₁, 𝐬₂) * kₜ(t₁, t₂);
 
     # Establish Jordan Lake Domain
     domain = Vector{Vector{Float64}}()
     points = [[x, y] for y in ymesh for x in xmesh]
     for p in points
         push!(domain, p)
     end
     M = length(domain);
 
     𝐊ₘ = reshape([kₛ(domain[j], domain[i]) for i = 1:M for j = 1:M], (M, M));
 
     𝐊ₘʰᵃˡᶠ = cholesky(𝐊ₘ).L;
     # 𝐊ₘʰᵃˡᶠ = sqrt(𝐊ₘ)
 
     F = -λₜ;
     G = 1;
     H = σ * sqrt(2 * λₜ);
     Σ₀ = 1 / (2 * λₜ);
     𝐇ᵇᵃʳ = Diagonal(H * ones(M));
 
     # Simulate env_data for all i ∈ {1, ..., T / Δt}
     env_data = zeros(M, length(ts))
 
     Random.seed!(3) # Set seed for reproducibility
     𝐯ᵢ = sqrt(Σ₀) * randn(M)
     for i = 1:length(ts)
         𝐰ᵢᵗⁱˡᵈᵉ = sqrt(1 / (2 * λₜ) * (1 - exp(-2 * λₜ * Δt))) * randn(M)
         𝐯ᵢ = exp(-λₜ * Δt) * 𝐯ᵢ + 𝐰ᵢᵗⁱˡᵈᵉ
         𝐳ᵢ = H * 𝐯ᵢ
         env_data[:, i] = 𝐊ₘʰᵃˡᶠ * 𝐳ᵢ
     end
 
     # Reshape the data into a 3D array (spatial_x, spatial_y, temporal)
     W = reshape(env_data, length(xmesh), length(ymesh), length(ts));
 
     # return W
     
     X = xmesh
     Y = ymesh   
     T = range(ts[1], stop=ts[end], length=length(ts))
 
     return EnvDataST(X, Y, T, W)
 end

end # module SyntheticData