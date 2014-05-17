import unittest
from matvec import Vector, Matrix
import rigidbody

class VectorTests:
	
	def test_add(self):
		v1, orig_v1 = self.new_vector_and_copy()
		v2, orig_v2 = self.new_vector_and_copy()
		
		result = v1 + v2
		
		expected = [ orig_v1[i] + orig_v2[i] for i in range(self.vec_dim) ]
		self.assertEqual( result._e, expected )
		self.assertEqual( v1._e, orig_v1._e )
		self.assertEqual( v2._e, orig_v2._e )
	
	def test_sub(self):
		v1, orig_v1 = self.new_vector_and_copy()
		v2, orig_v2 = self.new_vector_and_copy()
		
		result = v1 - v2
		
		expected = [ orig_v1[i] - orig_v2[i] for i in range(self.vec_dim) ]
		self.assertEqual( result._e, expected )
		self.assertEqual( v1._e, orig_v1._e )
		self.assertEqual( v2._e, orig_v2._e )
		
	def test_inplace_add(self):
		v1, orig_v1 = self.new_vector_and_copy()
		v2, orig_v2 = self.new_vector_and_copy()
		
		v1 += v2
		
		expected = [ orig_v1[i] + orig_v2[i] for i in range(self.vec_dim) ]
		self.assertEqual( v1._e, expected )
		self.assertEqual( v2._e, orig_v2._e  )
		
	def test_inplace_sub(self):
		v1, orig_v1 = self.new_vector_and_copy()
		v2, orig_v2 = self.new_vector_and_copy()
		
		v1 -= v2
		
		expected = [ orig_v1[i] - orig_v2[i] for i in range(self.vec_dim) ]
		self.assertEqual( v1._e, expected )
		self.assertEqual( v2._e, orig_v2._e  )
		
	def test_dot(self):
		v1, orig_v1 = self.new_vector_and_copy()
		v2, orig_v2 = self.new_vector_and_copy()
		
		result = Vector.dot( v1, v2 )
		
		expected = sum([ orig_v1[i] * orig_v2[i] for i in range(self.vec_dim) ])
		self.assertEqual( result, expected )
		self.assertEqual( v1._e, orig_v2._e )
		self.assertEqual( v2._e, orig_v2._e  )
	
	def test_mul(self):
		v, orig_v = self.new_vector_and_copy()
		s = self.get_some_scalar()
		
		result = v * s
		
		self.assertEqual( result._e, [ orig_v[i]*s for i in range(self.vec_dim) ] )
		self.assertEqual( v._e, orig_v._e )
	
	def test_imul(self):
		v, orig_v = self.new_vector_and_copy()
		s = self.get_some_scalar()
		
		v *= s
		
		self.assertEqual( v._e, [ orig_v[i]*s for i in range(self.vec_dim) ] )
	
	def test_neg(self):
		v, orig_v = self.new_vector_and_copy()
		v = Vector( self.new_elem_list() )
		
		result = -v
		
		self.assertEqual( result._e, [ -orig_v[i] for i in range(self.vec_dim) ] )
		self.assertEqual( v._e, orig_v._e )
	
	# helpers
	@property
	def vec_dim(self): return len(self.new_elem_list())
	def new_vector_and_copy(self):
		els = self.new_elem_list()
		return ( Vector( els ), Vector( els ) )
	
	def get_some_scalar(self): return 5

class VectorTests2D(VectorTests, unittest.TestCase):

	def new_elem_list(self):
		return [1,2]
	
	def test_new_vector(self):
		v = Vector([3,4])
		self.assertEqual(v.dim, 2)
		self.assertEqual(v.x, 3)
		self.assertEqual(v.y, 4)
		self.assertEqual(v.norm(), 5.0)
		self.assertEqual([3,4], v._e)
	
	def test_cross(self):
		v1,v2 = self.new_vector_and_copy()
		try:
			v3 = Vector.cross(v1,v2)
			failed = False
		except: failed = True
		self.assertEqual(failed, True)

class VectorTestN(VectorTests, unittest.TestCase):
	def new_elem_list(self):
		return [1,2,3,4,5,6,7]

class VectorTests3D(VectorTests, unittest.TestCase):

	def new_elem_list(self):
		return [1,2,3]
	
	def test_new_vector(self):
		v = Vector([1,2,3])
		self.assertEqual(v.dim, 3)
		self.assertEqual(v.x, 1)
		self.assertEqual(v.y, 2)
		self.assertEqual(v.z, 3)
		self.assertEqual(v.norm()**2, float(1+4+9))
		self.assertEqual([1,2,3], v._e)

class MatrixTests(unittest.TestCase):
	
	def test_assignment_and_indexing(self):
		rowMajor = [[1,2,3],
		            [4,5,6]]
		
		M = Matrix.fromRowMajor( rowMajor )
		
		self.assertEqual( M.columns, 3 )
		self.assertEqual( M.rows, 2 )
		self.assertEqual( M[0,0], 1 )
		self.assertEqual( M[1,0], 4 )
		self.assertEqual( M[0,1], 2 )
		self.assertEqual( M[1,2], 6 )
		
		W = M.__copy__()
		
		self.assertEqual( M.rowMajorArrays(), rowMajor )
		self.assertEqual( rowMajor, W.rowMajorArrays() )
		
		M[1,0] = -5
		self.assertEqual( M[1,0], -5 ) 
		self.assertEqual( W[1,0], 4 )
		
	def test_transpose(self):
		rowMajor = [[1,2,3],
		            [4,5,6]]
		M1 = Matrix.fromRowMajor( rowMajor )
		
		M2 = M1.transpose()
		
		expected = [[1,4],
		            [2,5],
		            [3,6]]
		            
		self.assertEqual( expected, M2.rowMajorArrays() )
		self.assertEqual( rowMajor, M1.rowMajorArrays() )
		
	def test_identity(self):
		M1 = Matrix.identity(3)
		
		rowMajor = [[1,0,0],
		            [0,1,0],
		            [0,0,1]]
		            
		self.assertEqual( rowMajor, M1.rowMajorArrays() )
		
	def test_diagonal(self):
		
		M1 = Matrix.diag([1,2,3])
		
		rowMajor = [[1,0,0],
		            [0,2,0],
		            [0,0,3]]
		            
		self.assertEqual( rowMajor, M1.rowMajorArrays() )
	
	def test_sum(self):
		rowMajor = [[1,2,3],
		            [4,5,6]]
		M1 = Matrix.fromRowMajor( rowMajor )
		M2 = Matrix.fromRowMajor( rowMajor )
		
		M3 = M1 + M2
		
		expected = [[2,4,6],
		            [8,10,12]]
		            
		self.assertEqual( expected, M3.rowMajorArrays() )
		self.assertEqual( rowMajor, M1.rowMajorArrays() )
		self.assertEqual( rowMajor, M2.rowMajorArrays() )
		
		M1 += M2
		self.assertEqual( expected, M1.rowMajorArrays() )
		self.assertEqual( rowMajor, M2.rowMajorArrays() )
	
	def test_mul(self):
	
		m1 = [[1,2,3],
		      [4,5,6]]
		
		m2 = [[1,0],
		      [0,1],
		      [1,1]]
		
		expected = [[4,  5],
		            [10,11]]
		
		M1 = Matrix.fromRowMajor( m1 )
		M2 = Matrix.fromRowMajor( m2 )
		
		M3 = M1 * M2
		            
		self.assertEqual( expected, M3.rowMajorArrays() )
		self.assertEqual( m1, M1.rowMajorArrays() )
		self.assertEqual( m2, M2.rowMajorArrays() )
	
	def test_mul_vec(self):
	
		m1 = [[1,2,3],
		      [4,5,6]]
		M1 = Matrix.fromRowMajor( m1 )
		
		vec = Vector([1,0,1])
		
		result = M1 * vec
		self.assertEqual( Vector([4,10])._e, result._e )
		self.assertEqual( m1, M1.rowMajorArrays() )
	
	def test_mul_scalar(self):
	
		m1 = [[1,2,3],
		      [4,5,6]]
		M1 = Matrix.fromRowMajor( m1 )
		
		result = M1 * 3
		
		expected = [[3, 6, 9],
		            [12,15,18]]
		
		self.assertEqual( expected, result.rowMajorArrays() )
		self.assertEqual( m1, M1.rowMajorArrays() )

class RigidBodyTests:
	
	def assertVecsEqual( self, v1, v2 ):
		self.assertEqual( v1.__class__, Vector )
		self.assertEqual( v2.__class__, Vector )
		self.assertEqual( v1._e, v2._e )
	
	@property
	def space(self): return (self.__class__.Space)()
	
	@property
	def Particle(self): return self.__class__.Particle
	
	@property
	def RigidBody(self): return self.__class__.Particle
	
	def linearMotionTestFor( self, particle ):
		
		self.assertVecsEqual( particle.position, self.space.zeroVector )
		self.assertVecsEqual( particle.velocity, self.space.zeroVector )
		self.assertVecsEqual( particle.force, self.space.zeroVector )
		
		forceVector = self.space.zeroVector
		forceVector.x = 1
		
		particle.applyForceCm( forceVector )
		self.assertVecsEqual( particle.force, forceVector )
		
		dt = 0.5
		particle.move( dt )
		
		expectedVelocity = forceVector * (dt / particle.mass)
		self.assertVecsEqual( particle.force, self.space.zeroVector )
		self.assertVecsEqual( particle.velocity, expectedVelocity )
		self.assertVecsEqual( particle.position, expectedVelocity * dt )
		
		particle.move( dt )
		self.assertVecsEqual( particle.velocity, expectedVelocity )
		self.assertVecsEqual( particle.position, expectedVelocity * dt * 2 )
	
	def test_particle_linear_motion(self):
		self.linearMotionTestFor( self.Particle( 2.0 ) )
		
	def test_rigid_body_linear_motion(self):
		self.linearMotionTestFor( self.RigidBody( 2.0, self.someInverseInertia ) )

class Space2DTest(unittest.TestCase):
	
	def test_cross_product( self ):
		
		space = rigidbody.Space2D()
		
		v1 = Vector([1,0])
		v2 = Vector([0,2])
		
		self.assertEqual( space.vectorVectorCrossProduct( v1, v2 ), 2 )
		self.assertEqual( space.vectorVectorCrossProduct( v2, v1 ), -2 )
		self.assertEqual( space.vectorVectorCrossProduct( v1, v1 ), 0 )
		self.assertEqual( space.rotationVectorCrossProduct( -2, v1 )._e, Vector([0,-2])._e )
		self.assertEqual( space.rotationVectorDotProduct( 2, 3 ), 6 )

class RigidBodyTests2D( RigidBodyTests, unittest.TestCase ):
	Space = rigidbody.Space2D
	class Particle(rigidbody.Particle, Space): pass
	class RigidBody(rigidbody.RigidBody, Space): pass
	
	@property
	def someInverseInertia(self): return 1.0
	
	def newRigidBody(self):
		mass = 3
		inverseInertia = 2
		return self.RigidBody( mass, inverseInertia )
	
	def test_rigid_body_initial(self):
		
		rigidBody = self.newRigidBody()
		
		self.assertVecsEqual( rigidBody.velocity, Vector([0,0]) )
		self.assertVecsEqual( rigidBody.position, Vector([0,0]) )
		self.assertEqual( rigidBody.torque, 0.0 )
		self.assertEqual( rigidBody.L, 0.0 )
	
	def test_rigid_body_rotation_and_torque(self):
		
		torque = 2.0
		dt = 0.5
		
		rigidBody = self.newRigidBody()
		rigidBody.applyTorque( torque )
		
		self.assertEqual( rigidBody.torque, torque )
		
		rigidBody.integrateAppliedForces(dt)
		
		self.assertEqual( rigidBody.torque, 0.0 )
		self.assertVecsEqual( rigidBody.velocity, Vector([0,0]) )
		self.assertVecsEqual( rigidBody.position, Vector([0,0]) )
		self.assertVecsEqual( rigidBody.force, Vector([0,0]) )
		expectedL = dt * torque
		self.assertEqual( rigidBody.L, expectedL )
		self.assertEqual( rigidBody.R, 0.0 )
		
		rigidBody.integratePosition(dt)
		
		self.assertEqual( rigidBody.L, expectedL )
		self.assertEqual( rigidBody.R, rigidBody.inverseInertia * expectedL * dt )
	
	def test_rigid_body_apply_force(self):
		
		at = Vector([0, 2.0])
		force = Vector([-1.0, 0])
		dt = 0.5
		
		rigidBody = self.newRigidBody()
		rigidBody.applyForce( force, at )
		
		self.assertEqual( rigidBody.torque, 2.0 )
		self.assertVecsEqual( rigidBody.force, force )

class RigidBodyTests3D( RigidBodyTests, unittest.TestCase ):
	Space = rigidbody.Space3D
	class Particle(rigidbody.Particle, Space): pass
	class RigidBody(rigidbody.RigidBody, Space): pass
	
	@property
	def someInverseInertia(self): return Matrix.identity(3)*2
	
	def newRigidBody(self):
		mass = 3
		inverseInertia = self.someInverseInertia
		return self.RigidBody( mass, inverseInertia )
	
	def test_rigid_body_initial(self):
		
		rigidBody = self.newRigidBody()
		
		self.assertVecsEqual( rigidBody.velocity, Vector([0,0,0]) )
		self.assertVecsEqual( rigidBody.position, Vector([0,0,0]) )
		self.assertVecsEqual( rigidBody.torque, Vector([0,0,0]) )
		self.assertVecsEqual( rigidBody.L, Vector([0,0,0]) )
	
	def test_rigid_body_rotation_and_torque(self):
		
		torque = Vector([0,0,2.0])
		dt = 0.25
		
		rigidBody = self.newRigidBody()
		rigidBody.applyTorque( torque )
		
		self.assertVecsEqual( rigidBody.torque, torque )
		
		rigidBody.integrateAppliedForces(dt)
		
		self.assertVecsEqual( rigidBody.torque, Vector([0,0,0]) )
		self.assertVecsEqual( rigidBody.velocity, Vector([0,0,0]) )
		self.assertVecsEqual( rigidBody.position, Vector([0,0,0]) )
		self.assertVecsEqual( rigidBody.force, Vector([0,0,0]) )
		expectedL = torque * dt
		self.assertVecsEqual( rigidBody.L, expectedL )
		self.assertEqual( rigidBody.R._e, Matrix.identity(3)._e )
		
		rigidBody.integratePosition(dt)
		
		self.assertVecsEqual( rigidBody.L, expectedL )
		self.assertVecsEqual( Vector([0,0,1]), rigidBody.R.column(2) )
		self.assertVecsEqual( Vector([0,0,1]), rigidBody.R.transpose().column(2) )
		self.assertEqual( 0, Vector.dot( rigidBody.R.column(1), rigidBody.R.column(2) ) )
		
	
	def test_rigid_body_apply_force(self):
		
		at = Vector([0, 2.0, 0])
		force = Vector([-1.0, 0, 0])
		dt = 0.5
		
		rigidBody = self.newRigidBody()
		rigidBody.applyForce( force, at )
		
		self.assertVecsEqual( rigidBody.torque, Vector([0,0,2]) )
		self.assertVecsEqual( rigidBody.force, force )
		
		rigidBody.move(dt)
		
		self.assertVecsEqual( Vector([0,0,1]), rigidBody.R.column(2) )
		self.assertVecsEqual( Vector([0,0,1]), rigidBody.R.transpose().column(2) )

if __name__ == '__main__':
	unittest.main()
