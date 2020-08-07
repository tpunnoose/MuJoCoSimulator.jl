include("../src/MuJoCoSimulator.jl")
include(joinpath(@__DIR__, "../../WooferQP/src/Simulator", "XMLParser.jl"))
include(joinpath(@__DIR__, "../../WooferQP/src/Common", "QuadrupedDynamics.jl"))
include(joinpath(@__DIR__, "../../WooferQP/src/Common", "MPCControl/MPCControl.jl"))
include(joinpath(@__DIR__, "../../WooferQP/src/Common", "Utilities.jl"))
include(joinpath(@__DIR__, "../../WooferQP/src/Common", "Config.jl"))

using .MuJoCoSimulator
import .MPCControl

function woofer_test()
    yaml_path = joinpath(@__DIR__, "../src/Simulator/Simulator.yaml")
    simulator_yaml = YAML.load(open(joinpath(yaml_path)))

    woofer = WooferConfig()
    ParseXML(woofer)
    xml_path = yaml_path = joinpath(@__DIR__, "../src/Simulator/woofer_out.xml")

    # Initialize simulator object
    control_timestep = simulator_yaml["sim_data_dt"]
    buffer_timestep = simulator_yaml["buffer_update_dt"]
    sim = MuJoCoSimulator.Simulator{Float64, Int64}(xml_path, 26, 12, control_timestep, buffer_timestep)
    
    param = MPCControl.ControllerParams(Float64, Int64)

    control! = let param = param
        function (torques, q, q̇, t)
            MPCControl.control!(torques, q, q̇, t, param)
        end
    end

    MuJoCoSimulator.simulate(sim, control!)
end

woofer_test()