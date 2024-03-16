include "xgo_2d.lua"

--TRAJECTORY_BUILDER.pure_localization_trimmer = {
--  max_submaps_to_keep = 3,
--}


TRAJECTORY_BUILDER.pure_localization = true
POSE_GRAPH.optimize_every_n_nodes = 20

return options
