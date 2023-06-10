# All abaqus classes are in one file, because there was a problem calling modules.
#  -*- coding: mbcs -*-
# Do not delete the following import lines
from abaqus import *
from abaqusConstants import *
import __main__

'''
Use the following command to run the file in Abaqus terminal, or run it through the run script option in the GUI.

execfile(r'Z:\Abaqus_semester_project\Abaqus_file\Beam.py')
'''

# Setting the work directory.
workdir = 'Z:\Abaqus_semester_project\Abaqus_file'
import os
os.chdir(workdir)

# class that defines the data fed to the abaqus model
class InputData:
    def __init__(self, h_T, b_T, s_T,                                              # T_Section.
                 h_I, b_I, s_I, t_I,                                               # I_Section.
                 D, t_D,                                                           # Pipe_Section.
                 length, force, E, nue):                                           # Values needed for deflection.

        # initialize the parent class's constructor

        # Initializing the parameters of the model.
        try:
            # The values for the T_Section.
            self.h_T = float(h_T)                                                  # height of the T_section
            self.b_T = float(b_T)                                                  # length of the T_section flange.
            self.s_T = float(s_T)                                                  # Thickness of the T_section.

            # The values for the I_Section.
            self.h_I = float(h_I)                                                  # height of the I_section.
            self.b_I = float(b_I)                                                  #length of the I_section flange.
            self.s_I = float(s_I)                                                  # Thickness of the I_section web.
            self.t_I = float(t_I)                                                  # Thickness of the I_section flange.

            # The Values for the Pipe section.
            self.D = float(D)                                                      # Outer diameter of the Pipe section.
            self.t_D = float(t_D)                                                  # Thickness of the pipe section.

            # The values needed for calculating the deflection of the beam.
            self.length = float(length)                                            # Length of the beam in N.
            self.force = float(force)                                              # Force on the beam in N.
            self.E = float(E)                                                      # Young's modulus in Mpa.
            self.nue = float(nue)                                                  # piosson's ratio.
        except ValueError:
            print("Please check the input values.")


    # Checking the validity of the parameters
        if self.h_T <= 0:
            raise Exception("> Invalid height of T section : %f" % self.h_T)
        elif self.b_T <= 0:
            raise Exception("> Invalid length of the T section Flange: %f" % self.b_T)
        elif self.s_T <= 0:
            raise Exception("> Invalid length of the T section Flange: %f" % self.s_T)
        elif self.h_I <= 0:
            raise Exception("> Invalid height of I section : %f" % self.h_I)
        elif self.b_I <= 0:
            raise Exception("> Invalid length of the I section Flange: %f" % self.b_I)
        elif self.s_I <= 0:
            raise Exception("> Invalid length of the I section Flange: %f" % self.s_I)
        elif self.t_I <= 0:
            raise Exception("> Invalid length of the I section Flange: %f" % self.t_I)
        elif self.D <= 0:
            raise Exception("> Invalid Diameter for the pipe section: %f" % self.s_I)
        elif self.t_D <= 0:
            raise Exception("> Invalid thickness for the pipe section: %f" % self.t_I)

    #Setting helping values for the Section.
        self.x1 = self.b_I / 2
        self.y1 = self.h_I
        self.x2 = self.b_T / 2
        self.y2 = self.h_I + self.h_T
        self.x3 = (self.D / 2) + (self.s_I / 2)
        self.y3 = self.h_I + self.h_T - (self.D / 2)
        self.x4 = self.D + (self.s_I / 2)
        self.load = self.force / (self.length * self.b_I)

class Beam(InputData):
    def __init__(self):
        InputData.__init__(self,140.00, 140.00, 15.00,
                         80.0, 42.0, 3.90, 5.90,
                         48.30, 2.30,
                         4400.00, 39000.00, 210000.00, 0.30)

    def create_sectionsandparts(self):
        import section
        import regionToolset
        import displayGroupMdbToolset as dgm
        import part
        import material
        import assembly
        import step
        import interaction
        import load
        import mesh
        import optimization
        import job
        import sketch
        import visualization
        import xyPlot
        import displayGroupOdbToolset as dgo
        import connectorBehavior
        session.viewports['Viewport: 1'].setValues(displayedObject=None)
        s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__',
            sheetSize=1000.0)
        g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
        s.setPrimaryObject(option=STANDALONE)
        s.Line(point1=(0.0, 0.0), point2=(self.x1, 80.0))
        s.delete(objectList=(g[2], ))
        s.Line(point1=(0.0, 0.0), point2=(self.x1, 0.0))
        s.HorizontalConstraint(entity=g[3], addUndoState=False)
        s.Line(point1=(0.0, 0.0), point2=(-self.x1, 0.0))
        s.HorizontalConstraint(entity=g[4], addUndoState=False)
        s.ParallelConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
        s.Line(point1=(0.0, 0.0), point2=(0.0, self.y1))
        s.VerticalConstraint(entity=g[5], addUndoState=False)
        s.PerpendicularConstraint(entity1=g[3], entity2=g[5], addUndoState=False)
        s.Line(point1=(0.0, self.y1), point2=(self.x1, self.y1))
        s.HorizontalConstraint(entity=g[6], addUndoState=False)
        s.PerpendicularConstraint(entity1=g[5], entity2=g[6], addUndoState=False)
        s.Line(point1=(0.0, self.y1), point2=(-self.x1, self.y1))
        s.HorizontalConstraint(entity=g[7], addUndoState=False)
        s.PerpendicularConstraint(entity1=g[5], entity2=g[7], addUndoState=False)

        s.delete(objectList=(g[7], ))
        s.Line(point1=(0.0, self.y1), point2=(-self.x1, self.y1))
        s.HorizontalConstraint(entity=g[8], addUndoState=False)
        s.PerpendicularConstraint(entity1=g[5], entity2=g[8], addUndoState=False)
        s.Line(point1=(0.0, self.y1), point2=(0.0, self.y2))
        s.VerticalConstraint(entity=g[9], addUndoState=False)
        s.ParallelConstraint(entity1=g[5], entity2=g[9], addUndoState=False)

        s.Line(point1=(0.0, self.y2), point2=(self.x2, self.y2))
        s.HorizontalConstraint(entity=g[10], addUndoState=False)
        s.PerpendicularConstraint(entity1=g[9], entity2=g[10], addUndoState=False)
        s.Line(point1=(0.0, self.y2), point2=(-self.x2, self.y2))
        s.HorizontalConstraint(entity=g[11], addUndoState=False)
        s.PerpendicularConstraint(entity1=g[9], entity2=g[11], addUndoState=False)
        s.CircleByCenterPerimeter(center=(self.x3, self.y3), point1=(self.x4, self.y3))
        s.CircleByCenterPerimeter(center=(-self.x3, self.y3), point1=(-self.x4, self.y3))

        mdb.models['Model-1'].sketches.changeKey(fromName='__profile__',
            toName='Sketch-1')
        s.unsetPrimaryObject()
        s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__',
            sheetSize=200.0)
        g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
        s1.setPrimaryObject(option=STANDALONE)
        s1.sketchOptions.setValues(gridOrigin=(0.0, 110.0))
        s1.retrieveSketch(sketch=mdb.models['Model-1'].sketches['Sketch-1'])
        session.viewports['Viewport: 1'].view.fitView()
        p = mdb.models['Model-1'].Part(name='Part-1', dimensionality=THREE_D,
            type=DEFORMABLE_BODY)
        p = mdb.models['Model-1'].parts['Part-1']
        p.BaseShellExtrude(sketch=s1, depth=self.length)
        s1.unsetPrimaryObject()
        p = mdb.models['Model-1'].parts['Part-1']

        del mdb.models['Model-1'].sketches['__profile__']

    def create_properties(self):
        mdb.models['Model-1'].Material(name='Steel')
        mdb.models['Model-1'].materials['Steel'].Elastic(table=((self.E, self.nue), ))
        mdb.models['Model-1'].HomogeneousShellSection(name='T', preIntegrate=OFF,
            material='Steel', thicknessType=UNIFORM, thickness=self.s_T,
            thicknessField='', idealization=NO_IDEALIZATION,
            poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT,
            useDensity=OFF, integrationRule=SIMPSON, numIntPts=5)
        mdb.models['Model-1'].HomogeneousShellSection(name='O', preIntegrate=OFF,
            material='Steel', thicknessType=UNIFORM, thickness=self.t_D,
            thicknessField='', idealization=NO_IDEALIZATION,
            poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT,
            useDensity=OFF, integrationRule=SIMPSON, numIntPts=5)
        mdb.models['Model-1'].HomogeneousShellSection(name='Web', preIntegrate=OFF,
            material='Steel', thicknessType=UNIFORM, thickness=self.s_I,
            thicknessField='', idealization=NO_IDEALIZATION,
            poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT,
            useDensity=OFF, integrationRule=SIMPSON, numIntPts=5)
        mdb.models['Model-1'].HomogeneousShellSection(name='Flange', preIntegrate=OFF,
            material='Steel', thicknessType=UNIFORM, thickness=self.t_I,
            thicknessField='', idealization=NO_IDEALIZATION,
            poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT,
            useDensity=OFF, integrationRule=SIMPSON, numIntPts=5)

        p = mdb.models['Model-1'].parts['Part-1']
        f = p.faces
        faces = f.getSequenceFromMask(mask=('[#603 ]', ), )
        region = p.Set(faces=faces, name='Set-1')
        p = mdb.models['Model-1'].parts['Part-1']
        p.SectionAssignment(region=region, sectionName='T', offset=0.0,
            offsetType=BOTTOM_SURFACE, offsetField='',
            thicknessAssignment=FROM_SECTION)
        p = mdb.models['Model-1'].parts['Part-1']
        f = p.faces
        faces = f.getSequenceFromMask(mask=('[#4 ]', ), )
        region = p.Set(faces=faces, name='Set-2')
        p = mdb.models['Model-1'].parts['Part-1']
        p.SectionAssignment(region=region, sectionName='T', offset=0.0,
            offsetType=MIDDLE_SURFACE, offsetField='',
            thicknessAssignment=FROM_SECTION)

        p = mdb.models['Model-1'].parts['Part-1']
        f = p.faces
        faces = f.getSequenceFromMask(mask=('[#820 ]', ), )
        region = p.Set(faces=faces, name='Set-3')
        p = mdb.models['Model-1'].parts['Part-1']
        p.SectionAssignment(region=region, sectionName='O', offset=0.0,
            offsetType=MIDDLE_SURFACE, offsetField='',
            thicknessAssignment=FROM_SECTION)
        p = mdb.models['Model-1'].parts['Part-1']
        f = p.faces
        faces = f.getSequenceFromMask(mask=('[#1d0 ]', ), )
        region = p.Set(faces=faces, name='Set-4')
        p = mdb.models['Model-1'].parts['Part-1']
        p.SectionAssignment(region=region, sectionName='Flange', offset=0.0,
            offsetType=MIDDLE_SURFACE, offsetField='',
            thicknessAssignment=FROM_SECTION)
        p = mdb.models['Model-1'].parts['Part-1']
        f = p.faces
        faces = f.getSequenceFromMask(mask=('[#8 ]', ), )
        region = p.Set(faces=faces, name='Set-5')
        p = mdb.models['Model-1'].parts['Part-1']
        p.SectionAssignment(region=region, sectionName='Web', offset=0.0,
            offsetType=MIDDLE_SURFACE, offsetField='',
            thicknessAssignment=FROM_SECTION)


    def create_instance(self):
        a = mdb.models['Model-1'].rootAssembly
        a = mdb.models['Model-1'].rootAssembly
        a.DatumCsysByDefault(CARTESIAN)
        p = mdb.models['Model-1'].parts['Part-1']
        a.Instance(name='Part-1-1', part=p, dependent=OFF)
    def create_step(self):
        mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial')

    def define_load(self):
        a = mdb.models['Model-1'].rootAssembly
        s1 = a.instances['Part-1-1'].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#3 ]', ), )
        side2Faces1 = s1.getSequenceFromMask(mask=('[#600 ]', ), )
        region = a.Surface(side1Faces=side1Faces1, side2Faces=side2Faces1,
            name='Surf-1')
        mdb.models['Model-1'].Pressure(name='Load-1', createStepName='Step-1',
            region=region, distributionType=UNIFORM, field='', magnitude=self.load,
            amplitude=UNSET)

    def Bcs(self):
        a = mdb.models['Model-1'].rootAssembly
        e1 = a.instances['Part-1-1'].edges
        edges1 = e1.getSequenceFromMask(mask=('[#49c2498 #63 ]', ), )
        region = a.Set(edges=edges1, name='Set-1')
        mdb.models['Model-1'].EncastreBC(name='BC-1', createStepName='Step-1',
            region=region, localCsys=None)

    def create_mesh(self):
        a = mdb.models['Model-1'].rootAssembly
        e1 = a.instances['Part-1-1'].edges
        pickedEdges = e1.getSequenceFromMask(mask=('[#4902498 #3 ]', ), )
        a.seedEdgeByNumber(edges=pickedEdges, number=1, constraint=FINER)

        a = mdb.models['Model-1'].rootAssembly
        e1 = a.instances['Part-1-1'].edges
        pickedEdges = e1.getSequenceFromMask(mask=('[#c0000 #60 ]', ), )
        a.seedEdgeByNumber(edges=pickedEdges, number=5, constraint=FINER)

        a = mdb.models['Model-1'].rootAssembly
        e1 = a.instances['Part-1-1'].edges
        pickedEdges = e1.getSequenceFromMask(mask=('[#b1404925 ]', ), )
        a.seedEdgeByNumber(edges=pickedEdges, number=20, constraint=FINER)
        a = mdb.models['Model-1'].rootAssembly
        partInstances =(a.instances['Part-1-1'], )
        a.generateMesh(regions=partInstances)
    def submit_job(self):
        mdb.Job(name='Job-1', model='Model-1', description='', type=ANALYSIS,
            atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90,
            memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True,
            explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF,
            modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='',
            scratch='', resultsFormat=ODB)
        mdb.jobs['Job-1'].submit(consistencyChecking=OFF)
        session.mdbData.summary()
        o3 = session.openOdb(name='Z:/Abaqus_semester_project/Job-1.odb')
        session.viewports['Viewport: 1'].setValues(displayedObject=o3)
        session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
            CONTOURS_ON_DEF, ))
        session.viewports['Viewport: 1'].odbDisplay.setPrimaryVariable(
            variableLabel='U', outputPosition=NODAL, refinement=(INVARIANT,
            'Magnitude'), )
        session.viewports['Viewport: 1'].odbDisplay.setPrimaryVariable(
            variableLabel='U', outputPosition=NODAL, refinement=(COMPONENT, 'U2'),
            )

# Initializing the class and calling methods to create the model.

s = Beam()
s.create_sectionsandparts()
s.create_properties()
s.create_instance()
s.create_step()
s.define_load()
s.Bcs()
s.create_mesh()
s.submit_job()
