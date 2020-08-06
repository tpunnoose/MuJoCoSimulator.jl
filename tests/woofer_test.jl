include("../src/MuJoCoSimulator.jl")
include("XMLParser.jl")
include("../Common/QuadrupedDynamics.jl")
include("../Common/MPCControl/MPCControl.jl")
include("../Common/Utilities.jl")
include("../Common/Config.jl")

using .MuJoCoSimulator

function woofer_test()
    simulator_yaml = YAML.load(open(joinpath(yaml_path)))

    # Initialize update rates
    ODRIVE_DT = simulator_yaml["simulator"]["odrive_dt"]
    SIM_DATA_DT = simulator_yaml["simulator"]["sim_data_dt"]
    MOCAP_DT = simulator_yaml["simulator"]["mocap_dt"]
    BUFFER_UPDATE_DT = simulator_yaml["simulator"]["buffer_update_dt"]
    

    control! = let param = param
        function (torques, q, q̇)
            MPCControl.control!(torques, q, q̇, param)
        end
    end
end