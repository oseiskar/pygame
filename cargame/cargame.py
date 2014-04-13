#!/usr/bin/python

import pygame
import sys
import time
import random
import math
from OpenGL.GL import *
from OpenGL.GLU import *

# Visualization flags
USE_GLUT = True
if USE_GLUT:
	from OpenGL.GLUT import *

SCREEN_SIZE = (1024,768)
PERSPECTIVE = True

sys.path.append('../')

import rigidbody
from matvec import Vector

# Physical constants
STATIC_FRICTION = 1.1 
DYNAMIC_FRICTION = 0.7
GRAVITY = 9.8 
ROLLING_RESISTANCE = 0.1
AIR_RESISTANCE = 0.1

# Sensitivity of controls
STEERING_SENSITIVITY = 5.0
BRAKE_UP = 3.0
BRAKE_DOWN = 4.0
THROTTLE_UP = 1.0
THROTTLE_DOWN = 1.0

# Size of track
TRACK_SIZE = 1000

def drawQuad( vertices, color, elevation = 0.0 ):
	
	glColor( *color )
	glBegin(GL_QUADS)
	glNormal(0,0,-1)
	for v in vertices:
		glVertex(v.x,v.y, elevation)
	glEnd()

def drawPoint( p, color, size = 1.0 ):
	glPointSize( size )
	glColor( *color )
	glBegin(GL_POINTS)
	glVertex(p.x,p.y,0)
	glEnd()

def drawLocalPoint( body, p, color, size = 1.0 ):
	drawPoint( body.localToGlobal( p ), color, size )

def drawVec( p1, p2, color ):
	glColor( *color )
	glBegin(GL_LINES)
	glVertex(p1.x,p1.y,0.01)
	glVertex(p2.x,p2.y,0.01)
	glEnd()

def drawCircle3d( center3d, axis3d, radius, color, npoints = 32 ):
	axis3d.normalize()
	d1 = Vector.cross( axis3d, Vector([0,0,1]) ).direction()
	d2 = Vector.cross( axis3d, d1 )
	glColor(*color)
	glBegin(GL_TRIANGLE_FAN)
	glVertex( *center3d._e )
	for i in xrange(npoints+1):
		th = (i / float(npoints)) * 2*math.pi
		p = center3d + (d1*math.sin(th) + d2*math.cos(th))*radius
		glVertex( *p._e )
	glEnd()


class Track(rigidbody.Space2D):
	
	def __init__( self, origin, scale ):
		
		self.origin = origin
		self.width = 15
		self.scale = scale
		self.maxCurve = 0.3
		self.generatePoints()
		self.glList = None
	
	def randDir( self ):
		return Vector( [random.normalvariate(0,1) for i in [1,2]] ).direction()
	
	def generatePoints( self ):
		
		nControlPoints = 16
		
		r = 1.0
		ax1 = self.randDir()
		ax2 = self.rotationVectorCrossProduct( 1, ax1 )
		controlPoints = []
		
		for i in xrange(nControlPoints):
			rel = i / float(nControlPoints)
			theta = rel * 2.0 * math.pi
			r = random.normalvariate( 1, self.maxCurve )
			
			actualR = r * self.scale
			p = (ax1 * math.cos( theta ) + ax2 * math.sin( theta ) ) * actualR 
			controlPoints.append( p + self.origin )
		
		self.points = []
		curveResolution = 50
		for i in xrange(0,nControlPoints):
			cur = controlPoints[i]
			last = controlPoints[i-1]
			secondLast = controlPoints[i-2]
			
			p1 = (secondLast + last) * 0.5
			p2 = (cur + last) * 0.5
			
			for j in xrange(curveResolution):
				t = j / float(curveResolution)
				c1 = p1 * (1-t) + last * t
				c2 = last * (1-t) + p2 * t
				p = c1 * (1-t) + c2 * t
				self.points.append(p)
			
	
	def render(self):
		if self.glList == None:
			self.glList = 1 # GL list identifier, change if there are more lists
			glNewList( self.glList, GL_COMPILE )
			glBegin(GL_TRIANGLE_STRIP)
			glColor(0.4, 0.4, 0.4)
			for i in xrange(-1,len(self.points)):
				last = self.points[i-1]
				cur = self.points[i]
				direction = (cur - last).direction()
				perp = self.rotationVectorCrossProduct( 1, direction )
				p1 = cur + perp * self.width * 0.5
				p2 = cur - perp * self.width * 0.5
				glColor(0.8, 0.4, 0.4)
				glVertex(p1.x, p1.y, 0.001)
				glColor(0.4, 0.4, 0.8)
				glVertex(p2.x, p2.y, 0.001)
			glEnd()
		
			glBegin(GL_LINES)
			glColor(0.8, 0.8, 0.4)
			for p in self.points:
				glVertex(p.x,p.y, 0.004)
			glEnd()
			glEndList()
		else:
			glCallList( self.glList )

REAR,FRONT = (0,1)
AXES = (REAR,FRONT)

class Car(rigidbody.RigidBody, rigidbody.Space2D):
	
	def __init__( self ):
		
		width = 2.5 
		height = 2
		
		mass = 500.0
		inertia = 1/12.0 * mass * (width**2 + height**2) 
		
		rigidbody.RigidBody.__init__( self, mass, 1.0 / inertia )
		
		self.width = width
		self.height = height
		
		self.rearWeightRatio = 0.52
		
		self.maxSteeringAngle = math.pi / 2
		self.steeringAngle = 0.0
		self.targetSteeringAngle = 0.0
		self.curThrottle = 0.0
		self.curBrake = 0.0
		self.totalForces = [ Vector([0,0]) for ax in AXES ]
		
		self.sliding = { REAR: False, FRONT: False }
		
		self.slideLength = 0
		
		Constraint = rigidbody.SingleBodyRelativePointVelocityConstraint
		self.wheelConstraints = [ Constraint(self, self.axisPos(ax), self.globalAxisDir() ) for ax in AXES ]
	
	def move( self, dt, game ):
		
		saDelta = (self.targetSteeringAngle - self.steeringAngle) * dt * STEERING_SENSITIVITY
		self.steeringAngle += saDelta
		
		def clampedControl(what, control, up, down):
			if control: return min( what + up * dt, 1.0 )
			else: return max( what - down * dt, 0.0 )
		
		self.curThrottle = clampedControl( self.curThrottle, self.throttle, THROTTLE_UP, THROTTLE_DOWN )
		self.curBrake = clampedControl( self.curBrake, self.brake, BRAKE_UP, BRAKE_DOWN )
		if self.curThrottle > 0: self.curBrake = 0.0
		
		self.axisWeight = {
			REAR: self.mass * self.rearWeightRatio * GRAVITY,
			FRONT: self.mass * (1.0-self.rearWeightRatio) * GRAVITY
		}
		
		# compute weight transfer based on last time
		longForces = Vector.dot(self.totalForces[FRONT] + self.totalForces[REAR], self.globalRearDrivingDir())
		WT_RATIO = 0.05 / 2.0 # center of mass height / wheelbase
		weightTransfer = longForces * WT_RATIO * GRAVITY
		self.axisWeight[FRONT] -= weightTransfer
		self.axisWeight[REAR] += weightTransfer
		
		friction = {}
		maxAxisFriction = {}
		axisPos = {}
		axisVel = {}
		axisTravelDir = {}
		axisForces = {}
		for ax in AXES:
			if self.sliding[ax]: friction[ax] = DYNAMIC_FRICTION
			else: friction[ax] = STATIC_FRICTION
			maxAxisFriction[ax] = self.axisWeight[ax] * friction[ax]
			axisPos[ax] = self.localToGlobal( self.axisPos(ax) )
			axisVel[ax] = self.globalPointVelocity( axisPos[ax] )
			axisTravelDir[ax] = axisVel[ax].safeDirection()
			axisForces[ax] = Vector([0,0])
			
			# rolling resistance
			if self.velocity.norm() > 1.0:
				axisForces[ax] += -axisTravelDir[ax] * self.axisWeight[FRONT] * ROLLING_RESISTANCE
		
		BRAKE_COEFFICIENT = 0.9
		frictionLeft = lambda total,used: math.sqrt( 1.0 - min((used/total)**2, 1.0) ) * total
		
		if self.curBrake > 0:
			coeff = STATIC_FRICTION * BRAKE_COEFFICIENT * self.curBrake
			for ax in AXES:
				braking = min(self.axisWeight[ax] * coeff, maxAxisFriction[ax] )
				braking *= min( 1.0, axisVel[ax].norm() )
				force = - axisTravelDir[ax] * braking
				axisForces[ax] += force
				maxAxisFriction[ax] = frictionLeft(maxAxisFriction[ax], braking)
			
		elif self.curThrottle > 0:
			POWER = self.mass * 200.0
			
			throttleMagn = min(self.curThrottle * POWER / (self.velocity.norm() + 0.5), maxAxisFriction[REAR] * 0.90)
			throttleForce = self.globalRearDrivingDir() * throttleMagn
			axisForces[REAR] += throttleForce
			maxAxisFriction[REAR] = frictionLeft( maxAxisFriction[REAR], throttleMagn )
		
		axdir = self.globalAxisDir()
		
		for ax in AXES:
			self.applyForce( axisForces[ax], axisPos[ax] )
			self.wheelConstraints[REAR].noMoveDir = axdir
		
		# air resistance
		self.applyForceCm( -self.velocity * self.velocity.norm() * AIR_RESISTANCE )
		
		self.integrateAppliedForces(dt)
		
		if self.currentTurningRadius != None:
			c = self.turningCenter()
			p = self.localToGlobal( self.axisPos(FRONT) )
			self.wheelConstraints[FRONT].noMoveDir = (p - c).direction()
		
		for ax in AXES:
			self.wheelConstraints[ax].setAbsLambda( maxAxisFriction[ax] * dt )
		
		self.constraints = self.wheelConstraints
		
		vertices = self.vertexPositions()
		for v in vertices:
			n = game.crossedNormal( v )
			if n != None:
				c = rigidbody.SingleBodyPointVelocityConstraint(self, v, n)
				c.minLambda = 0.0
				self.constraints.append( c )
		
		for c in self.constraints:
			c.resetAndComputeJacobians()
		
		N_ITER = 10
		for i in range(N_ITER):
			for c in self.constraints:
				c.applyImpulses()
		
		for ax in AXES:
			self.totalForces[ax] = axisForces[ax] + self.wheelConstraints[ax].totalForce(dt)
			for side in (-1,1):
				wheel = self.localToGlobal( self.wheelPos( ax, side ) )
		
		self.integratePosition(dt)
		
		for ax in (FRONT,REAR):
			self.sliding[ax] = self.wheelConstraints[ax].lambdaActive()
	
	def turningCenter( self ):
		if self.currentTurningRadius == None: return None
		else:
			local = self.axisPos(REAR) + self.localAxisDir() * self.currentTurningRadius
			return self.localToGlobal( local )
	
	def wheelPos( self, ax, side ):
		return self.axisPos(ax) + self.localAxisDir() * (self.height * 0.5 * side)
	
	def globalAxisDir( self ):
		return self.localToGlobal( Vector([0,1]) ) - self.position
		
	def globalRearDrivingDir( self ):
		return self.localToGlobal( Vector([1,0]) ) - self.position
	
	def globalFrontDrivingDir( self ):
		tc = self.turningCenter()
		if tc == None: return self.globalRearDrivingDir()
		r = self.localToGlobal( self.axisPos(FRONT) ) - tc
		d = Vector( [-r.y, r.x] )
		if Vector.dot(d, self.globalRearDrivingDir()) < 0: d = -d
		return d
	
	def localAxisDir( self ):
		return Vector([0,1])
	
	@property
	def cmShift( self ):
		return -(self.rearWeightRatio - 0.5) * 2.0
	
	def axisPos( self, ax ):
		if ax == REAR: return Vector([-1 + self.cmShift, 0])
		elif ax == FRONT: return Vector([1 + self.cmShift, 0])
		
	
	def localVertexPositions( self ):
		hw = self.width * 0.5
		hh = self.height * 0.5
		return [
			Vector( [-hw,-hh ] ),
			Vector( [  hw,-hh ] ),
			Vector( [  hw, hh ] ),
			Vector( [ -hw, hh ] ) ]
	
	@property
	def currentTurningRadius( self ):
		if self.steeringAngle == 0.0:
			return None
		else:
			return -1.0 / math.tan(self.steeringAngle)
	
	def setControls( self, relx, rely, clickPos, buttons ):
		
		#rel = clickPos
		sa = (relx-0.5) * math.pi * 0.2
		self.targetSteeringAngle = min( max( sa, -self.maxSteeringAngle), self.maxSteeringAngle)
		
		self.throttle = buttons[0]
		self.brake = buttons[2]
		
		#forceMagn = (rely - 0.25) * 5000
		#globalForce = self.localToGlobal( Vector([1, 0])*forceMagn ) - self.position
		#self.applyForce( globalForce, self.localToGlobal( self.rearAxisPos() ) )
	
	def vertexPositions( self ):
		return [self.localToGlobal(v) for v in self.localVertexPositions()]
	
	def drawWheel( self, pos, frontWheel ):
		
		pos = self.localToGlobal( pos )
		if frontWheel and self.currentTurningRadius != None:
			tc = self.turningCenter()
			ax = (pos - tc).direction()
			direction = self.rotationVectorCrossProduct(1, ax).direction()
		else:
			direction = self.globalRearDrivingDir()
			ax = self.globalAxisDir()
		
		wheelR = self.width * 0.15
		col = (0.3,0.3,0.3)
		if USE_GLUT:
			glPushMatrix()
			glColor( *col )
			glTranslate( pos.x, pos.y, wheelR )
			glRotate( math.atan2( direction.y, direction.x ) / math.pi * 180.0, 0,0,1 )
			glRotate( 90, 1,0,0 )
			cylH = 0.3
			glTranslate( 0,0, -cylH*0.5 )
			glutSolidCylinder( wheelR, cylH, 16, 1 )
			glPopMatrix()
		else:
			drawCircle3d( Vector([pos.x,pos.y,wheelR]), Vector([ax.x,ax.y,0]), wheelR, col )
	
	def render(self):
		
		glPushMatrix()
		glTranslate( self.position.x, self.position.y, 0.0 )
		
		totalForces = self.totalForces[REAR] + self.totalForces[FRONT]
		sideForces = Vector.dot(totalForces, self.globalAxisDir() )
		frontForces = Vector.dot(totalForces, self.globalRearDrivingDir() )
		sideTilt = sideForces / self.mass * 1.1
		frontTilt = -frontForces / self.mass * 0.6
		
		glRotate( self.R / math.pi * 180, 0,0,1 )
		glRotate( sideTilt, 1,0,0 )
		glRotate( frontTilt, 0,1,0 )
		glTranslate( 0,0,0.8 )
		
		glColor( 0.8, 0.5, 0.5 )
		if USE_GLUT:
			glPushMatrix()
			depth = 0.5
			glScale(self.width, self.height, depth)
			glutSolidCube(1.0)
			glPopMatrix()
		else:
			glBegin(GL_QUADS)
			#glNormal(0,0,-1)
			for v in self.localVertexPositions():
				glVertex(v.x,v.y, 0)
			glEnd()
		
		def drawLight( value, offset, width, height, color ):
			glColor( *(color + (value,)) )
			if not USE_GLUT: return drawBar( value, offset, width )
			
			glEnable(GL_BLEND)
			glDisable(GL_LIGHTING)
			for side in [-1,1]:
				glPushMatrix()
				glTranslate( -self.width * 0.5, side * (self.height * offset * 0.5), 0 )
				glScale( 0.1, width, depth * height )
				glutSolidCube( 1.0 )
				glPopMatrix()
			glEnable(GL_LIGHTING)
			glDisable(GL_BLEND)
		
		def drawBar( value, offset, width, elevation = 0.01 ):
			if USE_GLUT:
				elevation += depth * 0.5
			hh = self.height * 0.5 * value
			hw = self.width * 0.5 * width
			w0 = self.width * (offset - 0.5)
			vert = [
				Vector( [-hw + w0,-hh, elevation ] ),
				Vector( [ hw + w0,-hh, elevation ] ),
				Vector( [ hw + w0, hh, elevation ] ),
				Vector( [-hw + w0, hh, elevation ] ) ]
			
			glNormal(0,0,1)
			glBegin(GL_QUADS)
			for v in vert:
				glVertex(*v._e)
			glEnd()
		
		# Draw throttle and brake bars
		drawLight( self.curBrake, 0.7, 0.3, 0.5, ( 1.0, 0.0, 0.0 ) ) 
		drawLight( self.curThrottle, 0.25, 0.4, 0.1, ( 1.0, 0.7, 0.0 ) ) 
		
		# Traction limit indicators
		glColor( 0.3, 0.3, 0.3 )
		frontTraction = self.wheelConstraints[FRONT].lambdaRatio()
		rearTraction = self.wheelConstraints[REAR].lambdaRatio()
		
		glColor( 0.5, 0.5, 0.5 )
		if frontTraction != None:
			drawBar( frontTraction, 0.9, 0.1, 0.02 ) 
		
		if rearTraction != None:
			drawBar( rearTraction, 0.1, 0.1, 0.02 ) 
		
		glPopMatrix()
		
		self.drawWheel( self.wheelPos( REAR, -1 ), False )
		self.drawWheel( self.wheelPos( REAR, 1 ), False )
		self.drawWheel( self.wheelPos( FRONT, -1 ), True )
		self.drawWheel( self.wheelPos( FRONT, 1 ), True )


class Game:

	def __init__(self):
		
		self.screenScale = 1.0/TRACK_SIZE 
		self.width = 1.0/self.screenScale
		self.height = SCREENTOP/self.screenScale
		
		self.track = Track( Vector([self.width/2, self.height/2]), self.width * 0.2 )
		
		self.car = Car()
		self.car.position = Vector( self.track.points[0] )
		self.T = 0
		self.cameraheading = 0.0
		
	
	def crossedNormal( self, point ):
		
		if point.x < 0: return Vector( [1,0] )
		if point.y < 0: return Vector( [0,1] )
		if point.x > self.width: return Vector( [-1,0] )
		if point.y > self.height: return Vector( [0,-1] )
		
		return None
	
	def move(self,dt,slo):
		
		self.T += dt
		self.car.move(dt, self)
		
		if self.car.velocity.norm() > 1.0:
			carDir = self.car.velocity
		else:
			carDir = self.car.globalRearDrivingDir()
		
		CAMERA_SPEED = 3.0
		# cameraheading = -self.car.R / math.pi * 180.0 + 90
		targetHeading = math.atan2( carDir.x, carDir.y ) / math.pi * 180.0
		headingDelta = ((targetHeading - self.cameraheading + 180) % 360) - 180
		self.cameraheading += headingDelta * dt * CAMERA_SPEED
		
		return True
			
	def mouse(self,xy,buttons):
		x,y = xy
		clickPos = Vector( [x,y] ) * (1.0 / self.screenScale)
		self.car.setControls( x,y, clickPos, buttons )
		
	def render(self):
		
		CAMERA_DISTANCE = 5
		camerapitch = -70
		
		glLoadIdentity()
		sc = self.screenScale
		glScale(sc, sc, sc)
		
		if PERSPECTIVE:
			# Transform to world coordinates by positioning camera
			glTranslate(0,-1,-CAMERA_DISTANCE)
		
			glRotate( camerapitch,  1,0,0 )
			glRotate( self.cameraheading,  0,0,1 )
			glTranslate(-self.car.position.x, -self.car.position.y, 0)
		
		glLight(GL_LIGHT0, GL_POSITION,  (1, 1, 3, 0))
		
		glEnable(GL_LIGHTING)
		glEnable(GL_TEXTURE_2D)
		
		glBegin(GL_QUADS)
		glNormal(0,0,1)
		vertices = [
			Vector( [0,0] ),
			Vector( [self.width,0 ] ),
			Vector( [self.width, self.height] ),
			Vector( [0, self.height] ) ]
		texScale = 0.1
		for v in vertices:
			glTexCoord2f(v.x*texScale,v.y*texScale);
			glVertex(v.x,v.y, 0.0)
		glEnd()
		
		glDisable(GL_TEXTURE_2D)
		glEnable(GL_COLOR_MATERIAL)
		
		self.track.render()
		self.car.render()
		
		print "\r", int(self.T),
		vel = self.car.velocity.norm()
		print int(vel * 3.6), "km/h",
		tr = self.car.currentTurningRadius
		if tr != None:
			realTr = (self.car.turningCenter() - self.car.position).norm()
			print int(realTr), 'm',
			g = vel*vel / realTr / GRAVITY
			print 'min %.2g m' % (vel*vel / (GRAVITY * STATIC_FRICTION)),
			print '%.3g g' % g,
		
		print "\t",
		sys.stdout.flush()
		
		#glEnable(GL_LIGHTING)
		

def resize(w, h):
	global SCREENTOP
	SCREENTOP = h/float(w)
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity()
	
	if PERSPECTIVE:
		FOVY = 60 # Field of view angle, y dimension
		NEARCLIP,FARCLIP = (2.0 / TRACK_SIZE, 1000 / float(TRACK_SIZE))
		gluPerspective(FOVY, float(SCREEN_SIZE[0])/SCREEN_SIZE[1], NEARCLIP, FARCLIP)
	else:
		gluOrtho2D(0, 1.0, 0, SCREENTOP)
		
	glMatrixMode(GL_MODELVIEW)
	

def initpygame():

	pygame.init()
	screen = pygame.display.set_mode(SCREEN_SIZE, pygame.DOUBLEBUF|pygame.OPENGL)
	pygame.display.set_caption("Car game")

	glShadeModel(GL_SMOOTH)
	glClearColor(0.5, 0.5, 0.5, 0.0)

	if USE_GLUT: glutInit()

	glEnable(GL_LIGHTING)
	glEnable(GL_LIGHT0)

	glEnable(GL_NORMALIZE)
	glEnable(GL_DEPTH_TEST)
	
	glBlendFunc(GL_SRC_ALPHA, GL_ONE)
	#glEnable(GL_CULL_FACE)
	#glCullFace(GL_BACK)

	glMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
	
	resize(*SCREEN_SIZE)
	
	ix = 2
	iy = 2
	col0 = '\xdd\xdd\xdd\xff'
	col1 = '\xcc\xcc\xcc\xff'
	image = col0 + col1 + col1 + col0
	
	textureId = glGenTextures(1)
	glBindTexture(GL_TEXTURE_2D, textureId)
	glPixelStorei(GL_UNPACK_ALIGNMENT,1)
	glTexImage2D(
		GL_TEXTURE_2D, 0, 3, ix, iy, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image
	)
	
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
	glBindTexture(GL_TEXTURE_2D, textureId)
	
	return screen

screen = initpygame()

ft = -1
def frame( fps ):
	optt = 1.0/fps
	minsleep = 0.001
	global ft
	if ft < 0:
		ft = time.time()
		return optt
	else:
		cur = time.time()
		diff = cur-ft
		sleept = optt-diff
		if sleept <  minsleep: sleept = minsleep
		time.sleep(sleept)
		cur = time.time()
		diff = cur-ft
		ft = cur
		return diff

game = Game()

def mouseXYFromEventPos( pos ):
	px = pos[0]/float(SCREEN_SIZE[0])
	py = (1.0-(pos[1] / float(SCREEN_SIZE[1])))*SCREENTOP
	return (px,py)

def mainLoop():
	
	while True:
		
		#dt = frame(20)
		dt = frame(50)  #* 3.0
		
		mouseDownPos = None
		
		slo = False
		for event in pygame.event.get():
				if event.type == pygame.QUIT:
						return
				elif event.type == pygame.KEYUP:
					if event.key == pygame.K_ESCAPE:
						return
				#elif event.type == pygame.MOUSEBUTTONDOWN:
				#	mouseDownPos = mouseXYFromEventPos(event.pos)
				#	game.click(*mouseDownPos)
		
		game.mouse( mouseXYFromEventPos( pygame.mouse.get_pos() ), pygame.mouse.get_pressed() )
		
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

		glLoadIdentity()

		if not game.move(dt,slo): return
		
		game.render()
		
		pygame.display.flip()

print ""
#pygame.event.set_grab(1)
mainLoop()
#pygame.event.set_grab(0)
