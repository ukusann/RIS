% Define enumeration for states
classdef states_enum< int8
    enumeration
        INIT_POS (1),
        GO_TO_TARGET (2),
        GO_TO_CHECKPOINT (3),
        GRIP_OPEN (4),
        GRIP_CLOSE (5),
        ERROR (-1)
    end
end