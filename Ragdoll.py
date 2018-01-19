# @file Ragdoll.py

import maya.cmds as mc
import Constraint as C
import RigidBody as RB

## This class contains the constraints and rigid bodies and performs the simulation loop
class Ragdoll(object):

	## Delete the geometry
	def reset(self):
		mc.delete("pCube1")
		mc.delete("pCube2")
		mc.delete("pCube3")
		mc.delete("pCube4")
		mc.delete("pCube5")
		mc.delete("pCube6")
		mc.delete("pCube7")
		mc.delete("pCube8")

	## Initialise the constraints and rigid bodies
	def start(self):
		self.c1 = C.Constraint("locator1")
		self.c2 = C.Constraint("locator2")
		self.c3 = C.Constraint("locator3")
		self.c4 = C.Constraint("locator4")
		self.c5 = C.Constraint("locator5")
		self.c6 = C.Constraint("locator6")
		self.c7 = C.Constraint("locator7")
		self.c8 = C.Constraint("locator8")
		self.c9 = C.Constraint("locator9")

		self.r1 = RB.RigidBody("pCube1", self.c1, self.c2, 0.5)
		self.r2 = RB.RigidBody("pCube2", self.c2, self.c3, 0.5)
		self.r3 = RB.RigidBody("pCube3", self.c1, self.c4, 0.5)
		self.r4 = RB.RigidBody("pCube4", self.c4, self.c5, 0.5)
		self.r5 = RB.RigidBody("pCube5", self.c1, self.c6, 0.5)
		self.r6 = RB.RigidBody("pCube6", self.c6, self.c7, 0.5)
		self.r7 = RB.RigidBody("pCube7", self.c1, self.c8, 0.5)
		self.r8 = RB.RigidBody("pCube8", self.c8, self.c9, 0.5)

	## Add forces to the rigid bodies
	def addForces(self):
		self.r2.addForce(30, [-0.25,4,-3.6])
		self.r4.addForce(30, [0.25,4,3.6])
		self.r6.addForce(30, [3.6, 4, 0.25])
		self.r8.addForce(30, [-2.4, 4, -0.25])

	## Perform the simulation loop
	# @param numFrames The number of frames to simulate
	def simulate(self, numFrames):
		for i in range(1, numFrames):
			# Update all the RigidBody
			for r in [self.r1, self.r2, self.r3, self.r4, self.r5, self.r6, self.r7, self.r8]:
				r.update()
			# Resolve all the constraints
			for r in [self.r1, self.r2, self.r3, self.r4, self.r5, self.r6, self.r7, self.r8]:
				r.resolveConstraints()
			# Set all the keys
			for r in [self.r1, self.r2, self.r3, self.r4, self.r5, self.r6, self.r7, self.r8]:
				r.setKeys(i)
			for c in [self.c1, self.c2, self.c3, self.c4, self.c5, self.c6, self.c7, self.c8, self.c9]:
				c.setKeys(i)
			# Reset the displacement variable for the constraints
			for c in [self.c1, self.c2, self.c3, self.c4, self.c5, self.c6, self.c7, self.c8, self.c9]:
				c.resetDisplacement()
