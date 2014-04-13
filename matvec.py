#
# Simple matrix and vector classes for 2D geometry
#
#	(c) Otto Seiskari, 2010-2014
#	email: otto.seiskari@gmail.com
#

import math

class Vector:
	
	def __init__(self, elem_list):
		self._e = elem_list[::]
		
	def __copy__(self): return Vector(self._e)

	def __mul__( self, m ): return Vector([m*e for e in self._e])
		
	def __imul__( self, m ):
		for i in range(self.dim): self._e[i] *= m
		return self
		
	def __add__( self, v ):
		return Vector([self[i]+v[i] for i in xrange(self.dim)])
		
	def __iadd__( self, v ):
		for i in range(self.dim): self._e[i] += v._e[i]
		return self
		
	def __isub__( self, v ):
		for i in range(self.dim): self._e[i] -= v._e[i]
		return self
		
	def __neg__( self ): return self*(-1.0)
		
	def __sub__( self, v ): return self + (-v)
	
	def __getitem__( self, i ): return self._e[i]
	
	def __setitem__( self, i, x ): self._e[i] = x
	
	@property
	def dim( self ): return len(self._e)
	
	@property
	def x( self ): return self._e[0]
	
	@property
	def y( self ): return self._e[1]
	
	@property
	def z( self ): return self._e[2]
	
	@staticmethod
	def dot( a, b ): return sum([a[i]*b[i] for i in range(a.dim)])
		
	@staticmethod
	def cross( a, b ):
		assert a.dim == 3 and b.dim == 3
		return Vector([
			a._e[1]*b._e[2] - a._e[2]*b._e[1],
			a._e[2]*b._e[0] - a._e[0]*b._e[2],
			a._e[0]*b._e[1] - a._e[1]*b._e[0]] )
	
	def norm( self ):
		return math.sqrt( Vector.dot(self,self) )
		
	def normalize( self ):
		self *= 1.0/self.norm()
	
	def direction( self ):
		v = Vector(self)
		v.normalize()
		return v
	
	def safeDirection( self ):
		v = Vector(self)
		l = v.norm()
		if l == 0.0: return v
		else: return v * (1.0/l)
	
	def __str__(self):
		return "%d-vector %s" % (self.dim, str(tuple(self._e)))

class Matrix:
	
	def __getitem__(self, ij):
		i,j = ij
		return self._e[j*self.rows+i]
	
	def __setitem__(self, ij, val):
		i,j = ij
		self._e[j*self.rows+i] = val
	
	def __init__( self, rows, columns ):
		self.rows = rows
		self.columns = columns
		self._e = [0.0]*(rows*columns)
	
	@staticmethod
	def fromRowMajor(rowMajorNestedArrays):
		rows = len(rowMajorNestedArrays)
		columns = len(rowMajorNestedArrays[0])
		
		r = Matrix(rows, columns)
		for i in range(rows):
			for j in range(columns):
				r[i,j] = rowMajorNestedArrays[i][j]
		
		return r
	
	@staticmethod
	def zero(rows, columns):
		return Matrix(rows, columns)
	
	@staticmethod
	def diag(array):
		n = len(array)
		r = Matrix( n, n )
		for i in xrange(len(array)): r[i,i] = array[i]
		return r
	
	@staticmethod
	def identity( n ):
		r = Matrix(n,n)
		for i in xrange(n): r[i,i] = 1.0
		return r
	
	def rowMajorArrays(self):
		r = []
		for i in range(self.rows):
			row = []
			for j in range(self.columns): row.append(self[i,j])
			r.append(row)
		return r
	
	def __copy__(self):
		r = Matrix(self.rows, self.columns)
		r._e = self._e[:]
		return r
	
	def __iadd__(self, other):
		assert self.columns == other.columns and self.rows == other.rows
		for i in range(len(self._e)): self._e[i] += other._e[i]
		return self
	
	def __add__(self, other):
		result = Matrix(self.rows, self.columns)
		for i in range(len(self._e)): result._e[i] = self._e[i] + other._e[i]
		return result
		
	def __mul__( self, m ):
		
		if isinstance( m, Vector ):
			assert m.dim == self.columns
			r = Vector([0] * self.rows)
			for i in range(self.rows):
				for j in range(self.columns):
					r[i] += self[i,j]*m[j]
			return r
			
		elif isinstance( m, Matrix ):
			assert m.rows == self.columns
			r = Matrix.zero(self.rows, m.columns)
			for k in range(r.columns):
				for i in range(r.rows):
					for j in range(self.columns):
						r[i,k] += self[i,j]*m[j,k]
			return r
		
		else: # should be scalar
			r = self.__copy__()
			for i in range(len(self._e)): r._e[i] *= m
			return r
			
	def column( self, i ):
		return Vector( [self[j,i] for j in xrange(self.rows)] )
			
	def transpose( self ):
		r = Matrix(self.columns, self.rows)
		for i in range(self.rows):
			for j in range(self.columns):
				r[j,i] = self[i,j]
		return r
	
	def __str__(self):
		return "%dx%d matrix %s" % (self.rows, self.columns, str(self.rowMajorArrays()))

