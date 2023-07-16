# ClassifiedObject.msg
# An array of confidence scores (one per class/option defined in the model being used to classifiy the object)
std_msgs/Float64MultiArray obj_class_score_array

# The x coordinate of the Bounding box used to classify the object
float64 bbox_center_x

# The y coordinate of the Bounding box used to classify the object
float64 bbox_center_y

# The x dimension (width) of Bounding box bbox used to classify the object
float64 bbox_dimension_x

# The y dimension (width) of Bounding box bbox used to classify the object
float64 bbox_dimension_y