# Object
# A single detected object label (with possible multiple instances in the image)

string object_name
ObjectInstance[] instances
string[] parents
float64 confidence          # in [0,100] -- percentage
