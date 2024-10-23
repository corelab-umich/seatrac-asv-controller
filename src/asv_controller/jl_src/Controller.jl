module Controller

using LinearAlgebra, Random, Statistics, StaticArrays, Interpolations, LazySets, SpatiotemporalGPs, LinearInterpolations, StatsBase

include("jordan_lake_domain.jl")
include("kf.jl")
include("ngpkf.jl")
include("ergodic.jl")
include("Convex_bound_avoidance.jl")

struct BoatState
    position_x::Float64
    position_y::Float64
    heading::Float64
    state_of_charge::Float64
end

struct Waypoint
    times::Vector{Float64}
    positions_x::Vector{Float64}
    positions_y::Vector{Float64}
end

struct ControlInput
    forward_velocity::Float64 # [m/s]
    heading_angle::Float64 # [rad] - relative to +x axis, CCW
end

struct InformationEstimate
    # grid::Grid
    wind_x::Matrix
    wind_y::Matrix
end

sigma_t = 2.0
sigma_s = 1.0
lt = 0.75 * 60.0 # minutes
ls = 0.75 # km
kern = Matern(1/2, sigma_s, ls)
dt = 2.5

# function create_target_q_matrix(xs, ys, target_q_mat)
#     for i in 1:length(xs)
#         for j in 1:length(ys)
#             p = [xs[i], ys[j]]
#             if p ∈ JordanLakeDomain.convex_polygon.polygon
#                 target_q_mat[i, j] = 0.95
#             end
#         end
#     end
# end

function Cfun(p, x)
    return kern(x, p)^2 / kern(p, p)
end

function Rfun(p, x)
    return (kern(x,x) - kern(x, p)^2 / kern(p, p) + 0.5^2)/(dt)
end

C_ = Cfun(0,0)
R_ = Rfun(0,0)

k = (C_^2 / R_)

function Clarity_delta_t(current_clarity, target_clarity)
    delta_t = (target_clarity - current_clarity) / ((target_clarity - 1) * k * (current_clarity - 1))
    return delta_t
end

function Clarity_delta_new(current_clarity, target_clarity)
    den = -target_clarity*k + k*current_clarity*target_clarity + k - k*current_clarity
    return delta_t = (target_clarity - current_clarity) / den
end

function ergo_controller_weighted_2(xs, Mean, w_rated_val, convex_polygon, target_q, Nx, Ny, x_domain, y_domain;
    ergo_grid,
    ergo_q_map,
    traj,
    umax= 0.15, #30.0 * 60 / 1000,
    ΔT,
    kwargs...
    )
    
    # Set the rated value matrix    
    w_rated = ones(Nx, Ny)
    w_rated *= w_rated_val

    #   Compute the target matrix 
    lambda_param = 0.05
    delta = -lambda_param*((Mean - w_rated).^2)
    q_target_temp = target_q*(exp.(delta)) 


    # Mask the matrix such that q_target is zero outisde the domain
    for i in 1:length(x_domain)
        for j in 1:length(y_domain)
            p = [x_domain[i], y_domain[j]]
            if (p ∈ convex_polygon.polygon) == false
                q_target_temp[i, j] = 0.0
            end
        end
    end

    # Get it in the right shape
    q_target_itp = linear_interpolation((x_domain, y_domain), q_target_temp, extrapolation_bc=Interpolations.Line())
    q_target_weighted = q_target_itp(ErgodicController.xs(ergo_grid), ErgodicController.ys(ergo_grid))     


    target_spatial_dist = zeros(size(ergo_q_map))
    Qp  = mean(σ_t.^2) # diagm(vec(σ_t .^2 * fuse_measurements_every_ΔT ))      

    for i in CartesianIndices(target_spatial_dist)
        if q_target_weighted[i] > ergo_q_map[i]
    #             println("here in first cond")
            target_spatial_dist[i] = Clarity_delta_new(ergo_q_map[i], q_target_weighted[i])
        else
            target_spatial_dist[i] = 0.0
        end
    end
        
    u = [ErgodicController.controller_single_integrator_cvx_bound(ergo_grid, x, traj, target_spatial_dist, convex_polygon; umax=umax, do_boundary_correction=true) for x in xs]

    #     println(u)

    return u, q_target_temp

end

end # module Controller
