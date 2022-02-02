import numpy as np

def vector(x, y, z):
	list = [x, y, z]
	return np.array(list)

def unit_vector(vec):
	mag = np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
	return vec/mag
	
def mag(vec):
	return np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
	
def dot(vec1, vec2):
	return (vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2])
	
def angleBetweenVectors(vec1, vec2):
	num  = dot(vec1,vec2)
	denom = mag(vec1) * mag(vec2)
# 	Do not allow divide by zero
	if(denom == 0):
		return 0
	else:
		return np.arccos(num/denom)
	
# Rodrigues Formula
def rotate_around_arbitrary(v, k, theta):
	a = v * np.cos(theta) 
	b = (cross_vector(k,v)*np.sin(theta))
	c = k*dot(k,v)*(1-np.cos(theta))
	return a+b+c
	
def cross_vector(vec1, vec2):
	x = (vec1[1]*vec2[2]) - (vec1[2]*vec2[1])
	y = (vec1[2]*vec2[0]) - (vec1[0]*vec2[2])
	z = (vec1[0]*vec2[1]) - (vec1[1]*vec2[0])
	return(vector(x, y, z))
