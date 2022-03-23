#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Kiran Prasad", "Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

from gen_maze import MazeObject

class DAEBounding(MazeObject):
	def __init__(self, coords=None, grid_dims=None, scale=1, out_xml=None, dae=None,name='dae_bounding',id=0, static=True):
		super(DAEBounding, self).__init__(coords=coords,
						 grid_dims=grid_dims,
						 scale=scale,
						 out_xml=out_xml,
						 name=name,
						 dae=dae,
						 id=id,
						 static=static)

	def add_object(self):
		super(DAEBounding, self).add_object()

	def add_object_description(self):
		super(DAEBounding, self).add_object_description()

class Table(MazeObject):
	def __init__(self, coords=None, grid_dims=None, scale=(1,1,1), out_xml=None, dae=None,name='table',id=0, static=True):
		super(Table, self).__init__(coords=coords,
						 grid_dims=grid_dims,
						 scale=scale,
						 out_xml=out_xml,
						 name=name,
						 dae=dae,
						 id=id,
						 static=static)

	def add_object(self):
		super(Table, self).add_object()

	def add_object_description(self):
		super(Table, self).add_object_description()   

class Cake(MazeObject):
    def __init__(self, coords=None, grid_dims=None, scale=(1,1,1), out_xml=None, dae=None,name='Croissant',color=None,id=0, static=False):
        super(Cake, self).__init__(coords=coords,
                         grid_dims=grid_dims,
                         scale=scale,
                         out_xml=out_xml,
                         name=name,
                         dae=dae,
                         id=id,
						 static=static)

    def add_object(self):
        super(Cake, self).add_object()

    def add_object_description(self):
        super(Cake, self).add_object_description()

class Wall(MazeObject):
	'''
	Pose coordinates to set location
	grid size to set length of the wall
	scale to set the <scale> param
	'''
	def __init__(self, coords=None, grid_dims=None, scale=1, out_xml=None, dae=None,id=0, static=True):
		self.x , self.y, self.z = coords
		self.scale = scale[0]
		self.grid_dim = grid_dims[0]
		self.length = self.scale * self.grid_dim
		self.f_out = out_xml
	
	def add_object(self):
		scale = (self.length+2)/7.5
		wall_dimensions = [(-1, self.length/2, -1.55905, scale, 1),
						   (self.length/2, self.length+1, 0, scale, 1),
						   (self.length+1, self.length/2, -1.55905, scale, 1),
						   (self.length/2, -1, 0, scale, 1)]

		for i in range(4):
			self.f_out.write('<model name=\'wall{}\'>\n'.format(i+1))
			self.f_out.write('<pose frame=\'\'>{} {} 0 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			self.f_out.write('<scale>{} {} 0.03</scale>\n'.format(wall_dimensions[i][3], wall_dimensions[i][4]))
			self.f_out.write('<link name=\'link\'>\n')
			self.f_out.write('<pose frame=\'\'>{} {} 0.42 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			self.f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')

	def add_object_description(self):
		for i in range(4):
			self.f_out.write('<model name=\'wall{}\'>\n'.format(i+1))
			self.f_out.write('<static>1</static>\n<link name=\'link\'>\n<pose frame=\'\'>0 0 0.42 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n')
			self.f_out.write('</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
			self.f_out.write('<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n</geometry>\n<material>\n<script>\n')
			self.f_out.write('<uri>model://grey_wall/materials/scripts</uri>\n<uri>model://grey_wall/materials/textures</uri>\n<name>vrc/grey_wall</name>\n</script>\n</material>\n</visual>\n<self_collide>0</self_collide>\n')
			self.f_out.write('<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>-0.779308 4.01849 0 0 -0 0</pose>\n</model>\n')
	
class Can(MazeObject):

	def __init__(self, coords=None, grid_dims=None, scale=1, out_xml=None, dae=None, id=0, static=True):
		self.x, self.y, self.z = coords
		self.f_out = out_xml
		self.id = id

	def add_object(self):
		self.f_out.write('<model name=\'can_{}\'>\n'.format(self.id))
		self.f_out.write('<pose frame=\'\'>{} {} -2e-06 1e-06 0 -9.5e-05</pose>\n'.format(self.x, self.y))
		self.f_out.write('<scale>0.5 0.5 1</scale>\n<link name=\'link\'>\n')
		self.f_out.write('<pose frame=\'\'>{} {} 0.114998 1e-06 0 -9.5e-05</pose>\n'.format(self.x, self.y))
		self.f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -3.822 0 -0 0</wrench>\n</link>\n</model>\n')

	def add_object_description(self):
		self.f_out.write('<model name=\'can_{}\'>\n'.format(self.id))
		self.f_out.write('<link name=\'link\'>\n<pose frame=\'\'>0 0 0.115 0 -0 0</pose>\n<inertial>\n<mass>0.39</mass>\n<inertia>\n<ixx>0.00058</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>0.00058</iyy>\n<iyz>0</iyz>\n<izz>0.00019</izz>\n</inertia>\n</inertial>\n<collision name=\'collision\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n')
		self.f_out.write('<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n<visual name=\'visual\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n<material>\n<script>\n<uri>model://beer/materials/scripts</uri>\n<uri>model://beer/materials/textures</uri>\n<name>Beer/Diffuse</name>\n</script>\n</material>\n</visual>\n')
		self.f_out.write('<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>0.888525 -2.58346 0 0 -0 0</pose>\n</model>\n')
		self.f_out.write('<gui fullscreen=\'0\'>\n<camera name=\'user_camera\'>\n<pose frame=\'\'>5 -5 2 0 0.275643 2.35619</pose>\n<view_controller>orbit</view_controller>\n<projection_type>perspective</projection_type>\n</camera>\n</gui>\n')

class Goal(MazeObject):
	def __init__(self, coords=None, grid_dims=None, scale=(1,1,1), out_xml=None, dae=None,name="goal",id=0, static=True):
		self.x, self.y, self.z = coords
		self.f_out = out_xml
		self.name = "goal"

	def add_object(self):
		self.f_out.write('<model name=\'goal\'>\n')
		self.f_out.write('<pose frame=\'\'>{} {} -9e-06 -1e-06 -4e-06 0</pose>\n'.format(self.x, self.y))
		self.f_out.write('<scale>1 1 1</scale>\n<link name=\'goal::link\'>\n')
		self.f_out.write('<pose frame=\'\'>{} {} 0.114991 -1e-06 -4e-06 0</pose>\n'.format(self.x, self.y))
		self.f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -9.8 0 -0 0</wrench>\n</link>\n</model>\n')
	
	def add_object_description(self):
		self.f_out.write('<model name=\'goal\'><static>0</static>\n<link name=\'link\'><pose frame=\'\'>0 0 0.115 0 -0 0</pose>\n<inertial>\n<mass>0.0005</mass><inertia>\n<ixx>0.00058</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>0.00058</iyy>\n')
		self.f_out.write('<iyz>0</iyz>\n<izz>0.00019</izz>\n</inertia>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n</inertial>\n<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n')
		self.f_out.write('<visual name=\'visual\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n<material>\n<script>\n<uri>model://beer/materials/scripts</uri>\n<uri>model://beer/materials/textures</uri>\n')
		self.f_out.write('<name>Beer/Diffuse</name>\n</script>\n<ambient>1 0 0 1</ambient>\n<diffuse>1 0 0 1</diffuse>\n<specular>0 0 0 1</specular>\n<emissive>0 0 0 1</emissive>\n<shader type=\'vertex\'>\n')
		self.f_out.write('<normal_map>__default__</normal_map>\n</shader>\n</material>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<cast_shadows>1</cast_shadows>\n<transparency>0</transparency>\n</visual>\n')
		self.f_out.write('<collision name=\'collision\'>\n<laser_retro>0</laser_retro>\n<max_contacts>10</max_contacts>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n')
		self.f_out.write('</cylinder>\n</geometry>\n<surface>\n<friction>\n<ode>\n<mu>1</mu>\n<mu2>1</mu2>\n<fdir1>0 0 0</fdir1>\n<slip1>0</slip1>\n<slip2>0</slip2>\n</ode>\n<torsional>\n<coefficient>1</coefficient>\n')
		self.f_out.write('<patch_radius>0</patch_radius>\n<surface_radius>0</surface_radius>\n<use_patch_radius>1</use_patch_radius>\n<ode>\n<slip>0</slip>\n</ode>\n</torsional>\n</friction>\n<bounce>\n')
		self.f_out.write('<restitution_coefficient>0</restitution_coefficient>\n<threshold>1e+06</threshold>\n</bounce>\n<contact>\n<collide_without_contact>0</collide_without_contact>\n<collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n')
		self.f_out.write('<collide_bitmask>1</collide_bitmask>\n<ode>\n<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n<kd>1</kd>\n<max_vel>0.01</max_vel>\n<min_depth>0</min_depth>\n</ode>\n')
		self.f_out.write('<bullet>\n<split_impulse>1</split_impulse>\n<split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n')
		self.f_out.write('<kd>1</kd>\n</bullet>\n</contact>\n</surface>\n</collision>\n</link>\n<static>0</static>\n<allow_auto_disable>1</allow_auto_disable>\n')
		self.f_out.write('<pose frame=\'\'>{} {} 0 0 -0 0</pose>\n</model>\n'.format(self.x, self.y))
