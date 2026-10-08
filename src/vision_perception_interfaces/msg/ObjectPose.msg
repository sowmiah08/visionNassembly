# One perceived object instance.
string name                         # object type, as in config/objects/<name>.yaml
geometry_msgs/PoseStamped pose      # object frame (= its SDF model frame) in `world`
float32 fitness                     # confidence: share of its points within 4 mm of the fitted model
float32 rmse                        # metres, over those points
int32 n_points                      # scene points used in the fit
float32 mask_score                  # SAM 2's own score for the mask
