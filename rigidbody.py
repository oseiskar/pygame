#
# Simple rigid body and particle classes for 2D and 3D motion
# Also contains some utilities for implementing a sequential
# impulse solver
#
#	(c) Otto Seiskari, 2010-2014
#	email: otto.seiskari@gmail.com
#

from matvec import Vector, Matrix
import math, cmath

class Space2D:
	
	@property
	def zeroVector(self): return Vector([0,0])
	
	@property
	def zeroRotationVector(self): return 0.0
	
	@property
	def identityRotation(self): return 0.0
	
	def normalizedRotation(self, R):
		return math.fmod(R, 2.0*math.pi)
	
	def inverseRotation( self, R ): return -R
	
	def applyRotation( self, R, point ):
		pointc = complex( point.x, point.y )
		newc = pointc * cmath.exp( complex(0,1) * R )
		return Vector( [newc.real, newc.imag] ) 
	
	def applyInfitesimalRotation( self, angle, angularVelocity, dt ):
		return angle + angularVelocity * dt
	
	def angMomToRotMat( self, angle, inverseInertia ):
		return inverseInertia
	
	def vectorVectorCrossProduct( self, v1, v2 ):
		return Vector.cross( Space2D.vecTo3D(v1), Space2D.vecTo3D(v2) ).z
	
	def rotationVectorCrossProduct( self, rotation, vec ):
		return Vector( [-vec.y, vec.x] ) * rotation
	
	def rotationVectorDotProduct( self, v1, v2 ):
		return v1*v2
	
	# helpers
	@staticmethod
	def vecTo3D( v ): return Vector( [v.x, v.y, 0] )

class Space3D:
	
	@property
	def zeroVector(self): return Vector([0,0,0])
	
	@property
	def zeroRotationVector(self): return self.zeroVector
	
	@property
	def identityRotation(self): return Matrix.identity(3)
	
	def normalizedRotation(self, R):
		Space3D.orthogonalize( R )
		return R
	
	def inverseRotation( self, R ): return R.transpose()
	
	def applyRotation( self, R, point ):
		return R*point
	
	def applyInfitesimalRotation( self, R, rot, dt ):
		# Infitesimal formula
		R = R + ( Space3D.crossProductMatrix( rot ) * R ) * dt
		return R
	
	def angMomToRotMat( self, R, inverseInertia):
		return R * inverseInertia * R.transpose()
	
	def vectorVectorCrossProduct( self, v1, v2 ):
		return Vector.cross( v1, v2 )
	
	def rotationVectorCrossProduct( self, rotation, vec ):
		return Vector.cross( rotation, vec )
	
	def rotationVectorDotProduct( self, v1, v2 ):
		return Vector.dot(v1,v2)
	
	# helpers
	@staticmethod
	def orthogonalize(m):
		"""
		Orthogonalize a matrix by applying Gram-Shmidt to
		its columns in ascending order
		"""
		
		c1 = m.column(0)
		c2 = m.column(1)
		c3 = m.column(2)
		
		c1.normalize()
		c2 += c1*(-Vector.dot(c2,c1))
		c2.normalize()
		c3 += c1*(-Vector.dot(c3,c1))
		c3 += c2*(-Vector.dot(c3,c2))
		c3.normalize()
		
		m._e = c1._e + c2._e + c3._e
	
	@staticmethod
	def crossProductMatrix(v):
		"""
		Returns  the cross product matrix V of vector v
		For any vector u, V satisfies: v x u = V u
		"""
		S = Matrix.zero(3,3)
		S[1,0] = v.z
		S[2,0] = -v.y
		S[0,1] = -v.z
		S[0,2] = v.y
		S[2,1] = v.x
		S[1,2] = -v.x
		return S

class Particle:
	"""Particle, a physical point object"""
	
	def __init__( self, mass ):
		
		self.mass = mass
		self.position = self.zeroVector
		self.velocity = self.zeroVector
		
		self.force = self.zeroVector
	
	def integrateAppliedForces( self, dt ):
		self.applyImpulseCm( self.force * dt )
		self.force *= 0
	
	def applyImpulseCm( self, impulse ):
		self.velocity += impulse * (1.0 / self.mass)
	
	def integratePosition( self, dt ):
		self.position += self.velocity * dt
	
	def move( self, dt ):
		"""
		Apporximates the state of this object after the given
		time interval dt using explicit Euler's method
		"""
		
		# Notice order
		self.integrateAppliedForces( dt )
		self.integratePosition( dt )
		
	def applyForceCm( self, F ):
		"""Applies force F to the center of mass of this object"""
		self.force += F

class RigidBody(Particle):
	"""Rigid body, a physical object"""
	
	def __init__( self, mass, inverseInertia ):
		
		Particle.__init__( self, mass )
		
		self.inverseInertia = inverseInertia
		self.R = self.identityRotation
		self.L = self.zeroRotationVector
		
		self.torque = self.zeroRotationVector
	
	def localToGlobal(self, localCoord):
		return self.position + self.applyRotation( self.R, localCoord )
	
	def globalToLocal(self, globalCoord):
		return self.applyRotation( self.inverseRotation(self.R), globalCoord - self.position )
	
	def applyImpulseTorque( self, impulseTorque ):
		self.L += impulseTorque
	
	def applyImpulse( self, linearImpulse, impulseTorque ):
		self.applyImpulseCm( linearImpulse )
		self.applyImpulseTorque( impulseTorque )
	
	def computeVelocityConstraintJacobians( self, point, noMoveDirection ):
		"""noMoveDirection should be normalized"""
		
		velocityPart = noMoveDirection
		badRotationDir = self.vectorVectorCrossProduct( point - self.position, noMoveDirection )
		# for symmetric inertia
		#precessionMatrix = self.angMomToRotMat( self.R, self.inverseInertia )
		angularMomentumPart = badRotationDir  #precessionMatrix*badRotationDir
		return (velocityPart, angularMomentumPart)
	
	def solveIsolatedImpulseLambda( self, velocityJacobian, angJacobian, bias = 0.0 ):
		em = Vector.dot( velocityJacobian, velocityJacobian ) / self.mass
		em += self.rotationVectorDotProduct(angJacobian, (self.inverseInertia * angJacobian))
		em = 1.0 / em
		deviation = Vector.dot( velocityJacobian, self.velocity )
		rot = self.angMomToRotMat( self.R, self.inverseInertia ) * self.L
		deviation += self.rotationVectorDotProduct( angJacobian, rot )
		return -em * (deviation + bias)
	
	def globalPointVelocity( self, point ):
		rel = point - self.position
		rot = self.angMomToRotMat( self.R, self.inverseInertia ) * self.L
		return self.rotationVectorCrossProduct( rot, rel ) + self.velocity
	
	def integrateAppliedForces( self, dt ):
		# Linear motion
		Particle.integrateAppliedForces( self, dt )
		
		# Rotation
		self.applyImpulseTorque( self.torque * dt )
		self.torque *= 0
	
	def integratePosition( self, dt ):
		# Linear motion
		Particle.integratePosition( self, dt )
		
		# Rotation
		rot = self.angMomToRotMat( self.R, self.inverseInertia ) * self.L
		self.R = self.applyInfitesimalRotation( self.R, rot, dt )
		self.R = self.normalizedRotation( self.R )
		
	def applyTorque( self, t ):
		"""Applies torque t to this body"""
		
		self.torque += t
		
	def applyForce( self, F, p ):
		"""Applies force F to a point p of this body"""
		
		self.applyForceCm( F )
		self.applyTorque( self.vectorVectorCrossProduct(p-self.position, F) )

class SingleBodyConstraint:
	def __init__(self, body):
		self.body = body
		self.minLambda = None
		self.maxLambda = None
		self.bias = 0.0
	
	def setAbsLambda( self, absLambda ):
		self.minLambda = -absLambda
		self.maxLambda = absLambda
	
	def lambdaActive(self):
		if self.minLambda != None and self.totalLambda == self.minLambda: return True
		if self.maxLambda != None and self.totalLambda == self.maxLambda: return True
		return False
	
	def lambdaRatio(self):
		if self.minLambda < 0 and self.totalLambda < 0: return self.totalLambda / self.minLambda
		if self.maxLambda > 0 and self.totalLambda > 0: return self.totalLambda / self.maxLambda
		return None
	
	def totalForce(self, dt):
		return self.jacobians[0] * (self.totalLambda / dt)
	
	def resetAndComputeJacobians(self):
		self.totalLambda = 0.0
		self.computeJacobians()
	
	def applyImpulses(self):
		prevLambda = self.totalLambda
		curLambda = self.body.solveIsolatedImpulseLambda( *self.jacobians, bias = self.bias )
		self.totalLambda += curLambda
		if self.minLambda != None: self.totalLambda = max(self.minLambda, self.totalLambda)
		if self.maxLambda != None: self.totalLambda = min(self.maxLambda, self.totalLambda)
		
		deltaLambda = self.totalLambda - prevLambda
		self.body.applyImpulse( self.jacobians[0] * deltaLambda, self.jacobians[1] * deltaLambda )

class SingleBodyPointVelocityConstraint(SingleBodyConstraint):
	
	def __init__(self, body, point, noMoveDir):
		
		SingleBodyConstraint.__init__(self, body)
		self.point = point
		self.noMoveDir = noMoveDir
	
	def computeJacobians(self):
		self.jacobians = self.body.computeVelocityConstraintJacobians( self.point, self.noMoveDir )

class SingleBodyRelativePointVelocityConstraint(SingleBodyPointVelocityConstraint):
	
	def __init__(self, body, localPoint, noMoveDir):
		self.localPoint = localPoint
		SingleBodyPointVelocityConstraint.__init__(self, body, None, noMoveDir)
	
	def computeJacobians(self):
		self.point = self.body.localToGlobal( self.localPoint )
		self.jacobians = self.body.computeVelocityConstraintJacobians( self.point, self.noMoveDir )

class ConstantRotationConstraint2D(SingleBodyConstraint):

	def __init__(self, body, angularVelocity):
		
		SingleBodyConstraint.__init__(self, body)
		self.angularVelocity = angularVelocity
	
	def computeJacobians(self):
		linearJacobian = Vector([0,0])
		angularJacobian = 1
		self.bias = -self.angularVelocity
		self.jacobians = (linearJacobian, angularJacobian)


