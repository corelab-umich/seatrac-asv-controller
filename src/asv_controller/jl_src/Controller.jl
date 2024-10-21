module Controller

# using ErgodicController

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

function ergo_controller_weighted_2(t, xs, Mean, w_rated_val, convex_polygon;
    ergo_grid,
    ergo_q_map,
    traj,
    umax= 0.15, #30.0 * 60 / 1000,
    ΔT,
    kwargs...
    )
    
    target_q = 0.95

    # Set the rated value matrix    
    Nx, Ny = length(synthetic_data.xs), length(synthetic_data.ys)
    w_rated = ones(Nx, Ny)
    w_rated *= w_rated_val

    #   Compute the target matrix 
    lambda_param = 0.05
    delta = -lambda_param*((Mean - w_rated).^2)
    q_target_temp = target_q*(exp.(delta)) 


    # Mask the matrix such that q_target is zero outisde the domain
    # x_domain = range(0, 1.4, length=Nx)
    # y_domain = range(0, 6.5, length=Ny)
    x_domain = synthetic_data.xs
    y_domain = synthetic_data.ys

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
