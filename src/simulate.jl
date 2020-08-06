using LinearAlgebra
using StaticArrays
using Rotations
using DataStructures

import YAML

include("Simulator.jl")

function simulate(simulator::Simulator, control!)
    # Get model from local directory
    s = loadmodel(simulator.xml_path, 1200, 900)

    # Pre-allocate memory for actuator torques
    actuator_torques = zeros(simulator.m)

    # Mujoco data and model
    d = s.d
    m = s.m

    last_t = 0.0
    last_sim_data_t = 0.0
    last_mocap_t = 0.0
    last_buffer_update_t = 0.0

    BUFFER_UPDATE_DT = simulator.buffer_timestep
    CONTROL_DT = simulator.control_timestep

    # # CircularBuffer for simulating control latency
    # control_latency_frames = 1 # 1ms
    # control_buffer = CircularBuffer{Vector{Float64}}(control_latency_frames)
    # fill!(control_buffer, zeros(12))

    # # CircularBuffer for simulating mocap latency
    # mocap_latency_frames = 1 # 1ms
    # init_mocap_measurement = [0, 0, 0.32, 1, zeros(3)...]
    # mocap_buffer = CircularBuffer{Vector{Float64}}(mocap_latency_frames)
    # fill!(mocap_buffer, init_mocap_measurement)

    # # CircularBuffer for simulating measurement latency
    # joint_latency_frames = 1 # 1ms
    # init_joint_measurement = zeros(24)
    # joint_buffer = CircularBuffer{Vector{Float64}}(joint_latency_frames)
    # fill!(joint_buffer, init_joint_measurement)

    # Loop until the user closes the window
    MuJoCoSimulator.alignscale(s)
    println("Press enter to begin simulation...")
    readline(stdin)
    println("Starting sim.")

    while !GLFW.WindowShouldClose(s.window)
        ### basically sim step so things don't have to be defined multiple times
        if s.paused
            if s.pert[].active > 0
                mjv_applyPerturbPose(m, d, s.pert, 1)  # move mocap and dynamic bodies
                mj_forward(m, d)
            end
        else
            # Slow motion factor: 10x
            # Regular time needs a factor of 0.5
            factor = s.slowmotion ? 10.0 : 0.5

            # Advance effective simulation time by 1/refreshrate
            startsimtm = d.d[].time
            starttm = time()
            refreshtm = 1.0 / (factor * s.refreshrate)
            updates = refreshtm / m.m[].opt.timestep

            steps = round(Int, round(s.framecount + updates) - s.framecount)
            s.framecount += updates

            for i = 1:steps
                # clear old perturbations, apply new
                d.xfrc_applied .= 0.0

                if s.pert[].select > 0
                    mjv_applyPerturbPose(m, d, s.pert, 0) # move mocap bodies only
                    mjv_applyPerturbForce(m, d, s.pert)
                end

                t = d.d[].time

                # # Check if it's time to update the delay buffers for control, joint pos/vel, and mocap
                # if (t - last_buffer_update_t) > BUFFER_UPDATE_DT
                #     # Add lagged motion capture data
                #     push!(mocap_buffer, s.d.qpos[1:7])
                #     lagged_mocap = popfirst!(mocap_buffer)

                #     # Add lagged joint pos/vel data
                #     push!(joint_buffer, s.d.sensordata[7:30])
                #     lagged_joint = popfirst!(joint_buffer)

                #     # Add lagged torques
                #     push!(control_buffer, actuator_torques)
                #     lagged_control = popfirst!(control_buffer)

                #     last_buffer_update_t = t
                # end

                # Publish simulation data LCM message. Sim data does not include any latency effects
                if t - last_sim_data_t > CONTROL_DT
                    # ground truth states

                    # FIXME
                    q = SVector{simulator.n+1}([s.d.qpos[1:7]; s.d.sensordata[7:18]])
                    q̇ = SVector{simulator.n}([s.d.qvel[1:6]; s.d.sensordata[19:30]])

                    try
                        @time control!(actuator_torques, q, q̇, t)
                    catch err
                        for (exc, bt) in Base.catch_stack()
                           showerror(stdout, exc, bt)
                           println()
                        end

                        GLFW.DestroyWindow(s.window)

                        return
                    end

                    s.d.ctrl .= actuator_torques

                    last_sim_data_t = t
                end

                # # Publish mocap data at rate determined by MOCAP_DT. Includes latency
                # if (t - last_mocap_t) > MOCAP_DT
                #
                # end

                # s.d.ctrl .= lagged_control
                mj_step(s.m, s.d)

                # Break on reset
                (d.d[].time < startsimtm) && break
            end
        end

        render(s, s.window)
        GLFW.PollEvents()
    end

    GLFW.DestroyWindow(s.window)
end
