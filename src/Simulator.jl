struct Simulator{T, S}
    xml_path::String
    n::S
    m::S

    control_timestep::T
    buffer_timestep::T
end