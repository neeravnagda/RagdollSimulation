# @file Constraint.py

import maya.cmds as mc

## This class is used to store information about a constraint
class Constraint(object):

	## Initialise the constraint
	# @param _name The name of the Maya object used for the constraint
	def __init__(self, _name):
		self.c_name = _name
		self.m_position = mc.xform(self.c_name, query=True, translation=True)
		# Linear velocity is used as momentum, as there is no mass, the units kgm/s cannot apply
		self.m_linearVelocity = [0,0,0]
		# Store the displacement as a vector, this is reset each frame
		self.m_displacement = [0,0,0]

	## Set the keys for translation and rotation
	# @param _frame The frame number to set
	def setKeys(self, _frame):
		mc.setKeyframe(self.c_name, attribute="translate", t=[_frame], inTangentType="spline", outTangentType="spline")
		mc.setKeyframe(self.c_name, attribute="rotate", t=[_frame], inTangentType="spline", outTangentType="spline")

	## Query the position of this constraint
	def updatePosition(self):
		self.m_position = mc.xform(self.c_name, query=True, translation=True)

	## Add linear velocity to this joint
	# @param _velocity The new velocity to add as [velocity.x, velocity.y, velocity.z]
	def addLinearVelocity(self, _velocity):
		self.m_linearVelocity[0] += _velocity[0]
		self.m_linearVelocity[1] += _velocity[1]
		self.m_linearVelocity[2] += _velocity[2]

	## Apply damping to the linear velocity
	def dampLinearVelocity(self):
		self.m_linearVelocity[0] *= 0.99
		self.m_linearVelocity[1] *= 0.99
		self.m_linearVelocity[2] *= 0.99

	## Add a displacement vector to the displacement
	# @param _displacement The displacement to add [dX, dY, dZ]
	def addDisplacement(self, _displacement):
		self.m_displacement[0] += _displacement[0]
		self.m_displacement[1] += _displacement[1]
		self.m_displacement[2] += _displacement[2]

	## Reset the displacement to 0 at the end of each frame
	def resetDisplacement(self):
		self.m_displacement = [0,0,0]
