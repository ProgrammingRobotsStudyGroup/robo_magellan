# We will use Point to store the location of a cone
# x is the base mid-point of cone
# y is the minimum y pixel of the cone
# z is the minimum real depth of the location, if available
# Note: y is not the mid-point but the base 
int16 x
int16 y
int16 z

# w, h & d are half-spreads for x, y & z respectively
# i.e. cone spans [x-w/2, x+w/2] along x axis
# [y, y+h] along y axis (remember, y is min)
# and [z, z+d]
int16 w
int16 h
int16 d

# The are of the cone
float32 area

