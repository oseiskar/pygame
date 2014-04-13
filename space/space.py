#
# An example application for simple unconstrained 3D rigid body motion
#
#	(c) Otto Seiskari, 2010-2014
#	email: otto.seiskari@gmail.com
#

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import pygame
import time
import random

sys.path.append('../')
from matvec import *
from rigidbody import Particle, RigidBody, Space3D

def Vector3(x,y,z): return Vector([x,y,z])
def openGlAffineMapping4x4( matrix3x3, vec ):
	"""
	Create a 4x4 matrix corresponding to an affine mapping
	defined the given 3x3 linear map and traslation vector
	
	The return value can be fed to OpenGL
	"""
	col1 = matrix3x3.column(0)._e+[0]
	col2 = matrix3x3.column(1)._e+[0]
	col3 = matrix3x3.column(2)._e+[0]
	col4 = vec._e+[1]
	return col1 + col2 + col3 + col4 

# General settings
SCREEN_SIZE = (1024,768)
DESIREDFPS = 50 # Desired frames per second valuexc
WINDOW_CAPTION = "Space Motion"
DETAIL = 1 # Level of Detail, 0-2
random.seed( 1 )

# GL display lists for scene features
SHIP, ENV, SCENE = range(1,4)

class Smoke(Particle, Space3D):
	
	LIFETIME = 3
	COLOR = [1, 0.8, 0.3]
	DELAY = 0.2
	LOD = [5,10,20][DETAIL]
	
	def __init__( self, pos, vel ):
		Particle.__init__( self, 1.0 )
		self.position = pos
		self.velocity = vel
		self.op = 1
		self.t = self.LIFETIME
		
	def move( self, dt ):
		Particle.move(self,dt)
		
		self.t -= dt
		self.op = self.t/self.LIFETIME
		
		if self.op < 0: return False
		return True
	
	def render( self ):
		glPushMatrix()
		glTranslate( *self.position._e )
		glColor( *(self.COLOR + [self.op]) )
		glutSolidSphere( self.op * (1.0-self.op), Smoke.LOD, Smoke.LOD )
		glPopMatrix()
			
class Ship(RigidBody, Space3D):

	class Engine:

		def __init__( self, triggers, position, force ):
			self.keys = triggers
			self.position = position
			self.force = force
			
		def thrust( self, ship ):
			x = ship.R*self.position + ship.position
			F = ship.R*self.force
			ship.applyForce( F, x )
			return Smoke(x, F*(-1.0) + ship.velocity)

	INERTIA = [0.5, 1, 0.2]
	MASS = 3.0
	
	ENGINES = [
		Engine( [pygame.K_z], Vector3(0,1,-.5), Vector3(-1,0,0) ),
		Engine( [pygame.K_z], Vector3(0,-1,-.5), Vector3(1,0,0) ),
		Engine( [pygame.K_x], Vector3(0,-1,-.5), Vector3(-1,0,0) ),
		Engine( [pygame.K_x], Vector3(0,1,-.5), Vector3(1,0,0) ),
		Engine( [pygame.K_UP, pygame.K_SPACE], Vector3(0,1,-.5), Vector3(0,0,1) ),
		Engine( [pygame.K_DOWN, pygame.K_SPACE], Vector3(0,-1,-.5), Vector3(0,0,1) ),
		Engine( [pygame.K_LEFT, pygame.K_SPACE], Vector3(1,0,-1), Vector3(0,0,1) ),
		Engine( [pygame.K_RIGHT, pygame.K_SPACE], Vector3(-1,0,-1), Vector3(0,0,1) )
		]
		
	def __init__( self ):
		RigidBody.__init__( self, Ship.MASS, Matrix.diag(Ship.INERTIA) )
		
def mainLoop():
	
	# initialize timing and
	oldtime = time.time()
	relmousex = 0
	relmousey = 0
	smoke = []
	lastsmokeemit = oldtime
	
	GRAVITY = 30
	CAMERA_DISTANCE = 8
	LIGHT_POS_GL = (0, 5, -2, 0)
	
	ship = Ship()
	ship.position = Vector3(30,4,-3)
	ship.velocity = Vector3(0,0,6)

	while True:
		neednewframe = False

		events = [pygame.event.wait()] + pygame.event.get()
		for event in events:
			if event.type == pygame.QUIT:
					return
			elif event.type == pygame.KEYUP:
				if event.key == pygame.K_ESCAPE:
					return
			elif event.type == pygame.MOUSEMOTION:
				relmousex = 2.0*(event.pos[0]*1.0 / SCREEN_SIZE[0]) - 1.0
				relmousey = 2.0*(event.pos[1]*1.0 / SCREEN_SIZE[1]) - 1.0
				
			elif event.type == MYTIMER:
				neednewframe = True
				
		if neednewframe:
				
			# ------------ Input handling & motion
			
			# Measure time passed since last frame
			curtime = time.time()
			dt = curtime-oldtime
			oldtime = curtime
			
			# Keyboard handling
			keys = pygame.key.get_pressed()
			for e in Ship.ENGINES:
				if any([keys[k] for k in e.keys]):
					smoke_p = e.thrust( ship )
					
					emitd = curtime - lastsmokeemit
					if emitd == 0 or emitd > Smoke.DELAY:
						lastsmokeemit = curtime
						smoke.append( smoke_p )
				
			# Camera rotation
			cameraheading = relmousex*180 + 180
			camerapitch = relmousey*60
			
			# Gravity
			ship.applyForceCm( ship.position * (-GRAVITY*ship.mass / ship.position.norm()**2) )
										
			# Move
			ship.move( dt )
				
			# ------------ Renering
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
			glLoadIdentity()
			
			# Transform to "environment / background" coordinates
			# that move but do not rotate with camera
			glRotate( camerapitch,  1,0,0 )
			glRotate( cameraheading,  0,1,0 )
			glCallList(ENV) # Render environment
			glClear(GL_DEPTH_BUFFER_BIT) # Leave to background
			glLoadIdentity()
			
			# Transform to world coordinates by positioning camera
			glTranslate(0,0,-CAMERA_DISTANCE)
			glRotate( camerapitch,  1,0,0 )
			glRotate( cameraheading,  0,1,0 )
			glTranslate(*(-ship.position)._e)
			
			# Lighting
			glLight(GL_LIGHT0, GL_POSITION, LIGHT_POS_GL)
			
			# Render ship
			glPushMatrix()
			glMultMatrixf( openGlAffineMapping4x4( ship.R, ship.position ) )
			glCallList(SHIP)
			glPopMatrix()
			
			# Render "scene" (planets, debris)
			glCallList(SCENE)
			
			# Render smoke
			glEnable(GL_BLEND); glEnable(GL_CULL_FACE)
			newsmoke = []
			for s in smoke:
				if s.move( dt ):
					newsmoke.append( s )
					s.render()
			smoke = newsmoke
			glDisable(GL_BLEND); glDisable(GL_CULL_FACE)
			
			# Update display
			pygame.display.flip()


# Generate planets
def randomGaussianVector3( sigma=1 ):
	return Vector3(*[random.gauss(0,sigma) for i in range(3)])

def generateShipModel():
	
	SHIP_COLOR = (0.7,0.7,0.7)

	glNewList( SHIP, GL_COMPILE )
	glColor( *SHIP_COLOR )
	glPushMatrix(),
	glTranslate(0,0,.5); glScale(1,0.4,2); glutSolidCube( 1 )
	glPopMatrix()
	glPushMatrix()
	glScale(0.5,0.5,1)
	glTranslate(2,0,-.5); glutSolidCube( 1 )
	glTranslate(-4,0,0); glutSolidCube( 1 )
	glTranslate(2,2,0); glutSolidCube( 0.4 )
	glTranslate(0,-4,0); glutSolidCube( 0.4 )
	glPopMatrix()
	glEndList()

def generateScene():
	
	MAIN_PLANET_COLOR = (0.4, 0.7, 0.2)
	MAIN_PLANET_RADIUS = 10
	MAIN_PLANET_DETAIL = [10,40,100][DETAIL]
	NUM_OTHER_PLANETS = 15
	OTHER_PLANET_MAX_RADIUS = 10
	OTHER_PLANET_ST_DIST = 200
	OTHER_PLANET_DETAIL = [5,20,100][DETAIL]
	NUM_DEBRIS = [100,500,15000][DETAIL]
	DEBRIS_COLOR = (0.5,0.3,0.2)
	DEBRIS_DIST = 30
	DEBRIS_SCATTER = 4
	DEBRIS_SCALE = [2,1,0.5][DETAIL]
	
	glNewList( SCENE, GL_COMPILE )
	glColor( *MAIN_PLANET_COLOR )
	glutSolidSphere( MAIN_PLANET_RADIUS, MAIN_PLANET_DETAIL, MAIN_PLANET_DETAIL )
	for i in range(NUM_DEBRIS):
		glColor( *DEBRIS_COLOR )
		glPushMatrix()
		p = randomGaussianVector3(); p._e[1] = 0; p.normalize()
		glTranslate( *(p*random.gauss(DEBRIS_DIST,DEBRIS_SCATTER))._e )
		glRotate( 90, *randomGaussianVector3()._e )
		glScale( *[DEBRIS_SCALE*random.random() for i in range(3)] )
		glutSolidCube( random.random()*1 )
		glPopMatrix()
	for i in range(NUM_OTHER_PLANETS):
		glPushMatrix()
		glColor( *[random.random() for i in range(3)] )
		glTranslate( *(randomGaussianVector3(OTHER_PLANET_ST_DIST)._e) )
		glutSolidSphere( random.random()*OTHER_PLANET_MAX_RADIUS, OTHER_PLANET_DETAIL, OTHER_PLANET_DETAIL )
		#glutSolidTeapot( random.random()*OTHER_PLANET_MAX_RADIUS, OTHER_PLANET_DETAIL )
		glPopMatrix()
	glEndList()

def generateEnvironment():
	
	NUM_STARS = [500,1000,4000][DETAIL]
	STAR_SIZE = 1
	STAR_COV = Matrix.diag([1,0.3,1])
	CENTER_OF_UNIVERSE = Vector3(1,0,0)
	
	glPointSize( STAR_SIZE )
	glNewList( ENV, GL_COMPILE )
	glDisable(GL_LIGHTING)
	glBegin(GL_POINTS)
	for i in range(NUM_STARS):
		glColor( *([1.0-random.random()**2]*3) )
		p = STAR_COV*randomGaussianVector3() + CENTER_OF_UNIVERSE
		glVertex( *p._e )
	glEnd()
	glEnable(GL_LIGHTING)
	glEndList()

# PyGame initialization
MYTIMER = pygame.USEREVENT
pygame.init()
screen = pygame.display.set_mode(SCREEN_SIZE, pygame.DOUBLEBUF | pygame.OPENGL)
pygame.display.set_caption(WINDOW_CAPTION)
pygame.time.set_timer( MYTIMER, 1000/DESIREDFPS )

# GLUT initialization
glutInit()
glEnable(GL_NORMALIZE)

# GL settings, lights and shading
glShadeModel(GL_SMOOTH); glEnable(GL_COLOR_MATERIAL)
glMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
glEnable(GL_LIGHTING); glEnable(GL_LIGHT0)
glBlendFunc(GL_SRC_ALPHA, GL_ONE)
glEnable(GL_DEPTH_TEST)
glCullFace(GL_BACK)

# GL settings, perspective
FOVY = 60 # Field of view angle, y dimension
NEARCLIP,FARCLIP = (0.1, 1000.)
glMatrixMode(GL_PROJECTION)
glLoadIdentity()
gluPerspective(FOVY, float(SCREEN_SIZE[0])/SCREEN_SIZE[1], NEARCLIP, FARCLIP)
glMatrixMode(GL_MODELVIEW)

# Run game
generateEnvironment()
generateScene()
generateShipModel()
mainLoop()
