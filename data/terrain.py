import math

NUM_VERTS_X = 30
NUM_VERTS_Y = 30
totalVerts = NUM_VERTS_X*NUM_VERTS_Y
totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1)
offset = -50.0
TRIANGLE_SIZE = 1.
waveheight=0.1
gGroundVertices = [None] * totalVerts*3
gGroundIndices = [None] * totalTriangles*3

i=0

for i in range (NUM_VERTS_X):		
	for j in range (NUM_VERTS_Y):
		gGroundVertices[(i+j*NUM_VERTS_X)*3+0] = (i-NUM_VERTS_X*0.5)*TRIANGLE_SIZE
		gGroundVertices[(i+j*NUM_VERTS_X)*3+1] = (j-NUM_VERTS_Y*0.5)*TRIANGLE_SIZE
		gGroundVertices[(i+j*NUM_VERTS_X)*3+2] = waveheight*math.sin(float(i))*math.cos(float(j)+offset)
		                         
index=0
for i in range (NUM_VERTS_X-1):
	for j in range (NUM_VERTS_Y-1):
		gGroundIndices[index] = 1+j*NUM_VERTS_X+i
		index+=1
		gGroundIndices[index] = 1+j*NUM_VERTS_X+i+1
		index+=1
		gGroundIndices[index] = 1+(j+1)*NUM_VERTS_X+i+1
		index+=1
		gGroundIndices[index] = 1+j*NUM_VERTS_X+i
		index+=1
		gGroundIndices[index] = 1+(j+1)*NUM_VERTS_X+i+1
		index+=1
		gGroundIndices[index] = 1+(j+1)*NUM_VERTS_X+i
		index+=1

#print(gGroundVertices)
#print(gGroundIndices)

print("o Terrain")

for i in range (totalVerts):
	print("v "),
	print(gGroundVertices[i*3+0]),
	print(" "),
	print(gGroundVertices[i*3+1]),
	print(" "),
	print(gGroundVertices[i*3+2])
		
for i in range (totalTriangles):
	print("f "),
	print(gGroundIndices[i*3+0]),
	print(" "),
	print(gGroundIndices[i*3+1]),
	print(" "),
	print(gGroundIndices[i*3+2]),
	print(" ")
	

