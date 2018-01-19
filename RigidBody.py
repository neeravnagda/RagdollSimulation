## @file RigidBody.py
# This file is used for the RigidBody class

import math
import maya.cmds as mc
import maya.api.OpenMaya as om

def vectorLengthSquared(_vector):
	return _vector[0]*_vector[0] + _vector[1]*_vector[1] + _vector[2]*_vector[2]

def vectorLength(_vector):
	return math.sqrt(_vector[0]*_vector[0] + _vector[1]*_vector[1] + _vector[2]*_vector[2])

def dot(_v1, _v2):
	return _v1[0]*_v2[0] + _v1[1]*_v2[1] + _v1[2]*_v2[2]

def cross(_v1, _v2):
    x = (_v1[1] * _v2[2]) - (_v1[2] * _v2[1])
    y = (_v1[2] * _v2[0]) - (_v1[0] * _v2[2])
    z = (_v1[0] * _v2[1]) - (_v1[1] * _v2[0])
    return [x,y,z]

## This class creates a RigidBody for one bone
class RigidBody(object):

	## Initialise the RigidBody
	# @param _name The name of the RigidBody
	# @param _parent The parent constraint
	# @param _child The child constraint
	# @param _mass The mass of the object the joint is attached to
	def __init__(self, _name, _parent, _child, _mass):
		# Set the name of the object
		self.c_name = _name
		self.c_mass = _mass
		self.c_parent = _parent
		self.c_child = _child

		# Calculate the distance to the constraints to find the initial length of the joint
		constraintsDisplacement = [i-j for i,j in zip(self.c_child.m_position, self.c_parent.m_position)]
		self.c_length = vectorLength(constraintsDisplacement)

		# Create the cube
		mc.polyCube(name=self.c_name, sx=6, sy=6, sz=6, w=self.c_length*0.9, d=0.5, h=0.5, ch=False)

		# Get the dag path of this mesh
		selectionList = om.MGlobal.getSelectionListByName(self.c_name)
		iterator = om.MItSelectionList(selectionList, om.MFn.kDagNode)
		# Check if nothing is selected
		if iterator.isDone():
			print "Error finding mesh."
		else:
			self.c_dagPath = iterator.getDagPath()
			try:
				self.c_dagPath.extendToShape()
			except:
				pass

		# Make sure the joint is initialised in the centre of the joints and oriented correctly
		self.m_lastOrientation = [1,0,0]
		centre = [(i+j)/2.0 for i,j in zip(self.c_parent.m_position, self.c_child.m_position)]
		newOrientation = [i-j for i,j in zip(self.c_child.m_position, self.c_parent.m_position)]
		mMat = om.MQuaternion(om.MVector(self.m_lastOrientation), om.MVector(newOrientation)).asMatrix()
		orientMatrix = [mMat[i] for i in range(16)]
		# Move and rotate the RigidBody
		mc.xform(self.c_name, worldSpace=True, translation=[0,0,0])
		mc.xform(self.c_name, worldSpace=True, relative=True, matrix=orientMatrix)
		mc.xform(self.c_name, worldSpace=True, translation=centre)
		self.m_lastPosition = mc.xform(self.c_name, query=True, translation=True)
		self.m_lastOrientation = newOrientation

		# Store some information for angular momentum
		self.m_lastRotationAxis = om.MVector(0,0,0)
		self.m_lastRotationAngle = 0.0

	## Set the keys for translation and rotation
	# @param _frame The frame number to set
	def setKeys(self, _frame):
		mc.setKeyframe(self.c_name, attribute="translate", t=[_frame], inTangentType="spline", outTangentType="spline")
		mc.setKeyframe(self.c_name, attribute="rotate", t=[_frame], inTangentType="spline", outTangentType="spline")

	## Add a force to the RigidBody
	# @param _force The magnitude of the force to apply
	# @param _point The point of contact for the force
	def addForce(self, _force, _point):
		#------------------------------------------------------------------------------
		# Calculate the rotational acceleration
		#------------------------------------------------------------------------------
		meshFn = om.MFnMesh(self.c_dagPath)
		# Find the point of contact, and the normal to the point
		point = om.MPoint(_point)
		vertexPos, pid = meshFn.getClosestPoint(point, om.MSpace.kWorld)
		vertexNorm = meshFn.getVertexNormal(pid, False, om.MSpace.kWorld)
		vertexNorm.normalize()

		# Convert to lists as it is easier to manipulate
		vertexPosition = [vertexPos[i] for i in range(3)]
		# Invert the normal as we want to go into the mesh
		vertexNormal = [-vertexNorm[i] for i in range(3)]

		displacementToCentre = [i-j for i,j in zip(self.m_lastPosition, vertexPosition)]
		displacementDotNormal = dot(displacementToCentre, vertexNormal)
		parallelVector = [i * displacementDotNormal for i in vertexNormal]
		perpendicularVector = [i-j for i,j in zip(displacementToCentre, parallelVector)]
		distance = vectorLength(perpendicularVector)

		# If the distance is zero, there is no moment on the joint
		if distance != 0:
			# Calculate the angular velocity
			angularVelocity = math.sqrt(_force/(self.c_mass * distance))
			# Calculate an axis angle rotation matrix
			# 0.04 is the length of one frame: 1/25 seconds
			angle = -1 * math.radians(angularVelocity * 0.04)
			# Find the axis
			axis1Len = vectorLength(displacementToCentre)
			axis1 = [i/axis1Len for i in displacementToCentre]
			axis2 = vertexNormal
			axis = cross(axis1, axis2)

			# SLERP the last axis-angle rotation with the new one
			lastQuat = om.MQuaternion(self.m_lastRotationAngle, self.m_lastRotationAxis)
			currentQuat = om.MQuaternion(angle, om.MVector(axis))
			# Store this rotation for angular momentum
			self.m_lastRotationAxis, self.m_lastRotationAngle = om.MQuaternion.slerp(lastQuat, currentQuat, 0.5).asAxisAngle()

		#------------------------------------------------------------------------------
		# Calculate the linear acceleration
		#------------------------------------------------------------------------------
		#a = F/m
		linearAcceleration = _force / self.c_mass
		# dv = a * dt
		# dt = 0.04 = 1 frame, and halve the linear velocity as it is added to both joints equally
		# Therefore the constant 0.04 can be replaced with 0.02
		dV = linearAcceleration * 0.02
		# The last linear velocity is the total velocity from both constraints
		linearVelocity = [vertexNormal[i]*dV for i in range(3)]
		self.c_parent.addLinearVelocity(linearVelocity)
		self.c_child.addLinearVelocity(linearVelocity)

	## Update the simulation
	def update(self):
		# Update the position and orientation of the mesh
		self.updateTransform()
		# Rotate with angular momentum
		self.applyAngularMomentum()
		# Move with linear momentum
		self.applyLinearMomentum()
		# Update the position and orientation of the mesh
		self.updateTransform()

	## Update the orientation and position to match the parent and child constraints
	def updateTransform(self):
		# Update the mesh position and orientation
		newPos = [(i+j)/2.0 for i,j in zip(self.c_parent.m_position, self.c_child.m_position)]
		newOrientation = [i-j for i,j in zip(self.c_child.m_position, self.c_parent.m_position)]
		mMat = om.MQuaternion(om.MVector(self.m_lastOrientation), om.MVector(newOrientation)).asMatrix()
		orientMatrix = [mMat[i] for i in range(16)]
		mc.xform(self.c_name, worldSpace=True, translation=[0,0,0])
		mc.xform(self.c_name, worldSpace=True, relative=True, matrix=orientMatrix)
		mc.xform(self.c_name, worldSpace=True, translation=newPos)
		self.m_lastOrientation = newOrientation
		self.m_lastPosition = newPos

	## Rotate using angular momentum
	def applyAngularMomentum(self):
		# Only rotate if there is angular momentum
		if self.m_lastRotationAngle != 0:
			# Get the rotation matrix. Note this is a OpenMaya.MMatrix and needs to be converted to a list
			mMat = om.MQuaternion(self.m_lastRotationAngle, self.m_lastRotationAxis).asMatrix()
			rotationMatrix = [mMat[i] for i in range(16)]
			# The transformation needs to happen around the object centre, so translate to origin, rotate, translate back
			negativeTranslate = [-i for i in self.m_lastPosition]
			# Rotate the parent
			mc.xform(self.c_parent.c_name, relative=True, worldSpace=True, translation=negativeTranslate)
			mc.xform(self.c_parent.c_name, relative=True, worldSpace=True, matrix=rotationMatrix)
			mc.xform(self.c_parent.c_name, relative=True, worldSpace=True, translation=self.m_lastPosition)
			# Rotate the child
			mc.xform(self.c_child.c_name, relative=True, worldSpace=True, translation=negativeTranslate)
			mc.xform(self.c_child.c_name, relative=True, worldSpace=True, matrix=rotationMatrix)
			mc.xform(self.c_child.c_name, relative=True, worldSpace=True, translation=self.m_lastPosition)

			# Calculate the displacements
			parentPos = mc.xform(self.c_parent.c_name, query=True, translation=True)
			parentDisplacement = [i-j for i,j in zip(parentPos, self.c_parent.m_position)]
			self.c_parent.addDisplacement(parentDisplacement)
			childPos = mc.xform(self.c_child.c_name, query=True, translation=True)
			childDisplacement = [i-j for i,j in zip(childPos, self.c_child.m_position)]
			self.c_child.addDisplacement(childDisplacement)

			# Damp the angle
			self.m_lastRotationAngle *= 0.99

	## Move using linear momentum
	def applyLinearMomentum(self):
		linearVelocity = [i+j for i,j in zip(self.c_parent.m_linearVelocity, self.c_child.m_linearVelocity)]
		if linearVelocity != [0,0,0]:
			# dx = v * dt
			dX = [i*0.04 for i in linearVelocity]
			# Move the constraints
			mc.xform(self.c_parent.c_name, relative=True, worldSpace=True, translation=dX)
			mc.xform(self.c_child.c_name, relative=True, worldSpace=True, translation=dX)

			# Add the displacement to the constraints
			self.c_parent.addDisplacement(dX)
			self.c_child.addDisplacement(dX)

			# Damp the velocity
			self.c_parent.dampLinearVelocity()
			self.c_child.dampLinearVelocity()

		# Store the positions of the object, parent and child joints
		self.c_parent.updatePosition()
		self.c_child.updatePosition()

	## Make sure the constraints are not stretching too far
	def resolveConstraints(self):
		# Calculate the change in length of the joint
		constraintsDisplacement = [i-j for i,j in zip(self.c_child.m_position, self.c_parent.m_position)]
		lengthChange = vectorLength(constraintsDisplacement) - self.c_length
		# Check if the original length is not maintained
		if (lengthChange > 1e-36) or (lengthChange < -1e-36):
			# Calculate the ratio for how much the parent and child move towards the centre
			parentDisplacement = vectorLength(self.c_parent.m_displacement)
			childDisplacement = vectorLength(self.c_child.m_displacement)
			#normalize these values
			total = parentDisplacement + childDisplacement
			parentDisplacement /= total
			childDisplacement /= total
			# Multiply by the lengthChange as a constant to move
			parentDisplacement *= lengthChange
			childDisplacement *= lengthChange

			#Displacement to centre
			parentConstraint = [(i-j)*parentDisplacement for i,j in zip(self.m_lastPosition, self.c_parent.m_position)]
			childConstraint = [(i-j)*childDisplacement for i,j in zip(self.m_lastPosition, self.c_child.m_position)]

			mc.xform(self.c_parent.c_name, relative=True, worldSpace=True, translation=parentConstraint)
			mc.xform(self.c_child.c_name, relative=True, worldSpace=True, translation=childConstraint)
