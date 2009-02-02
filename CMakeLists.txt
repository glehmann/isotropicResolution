#Change PROJECT_NAME to the name of your project
PROJECT(isotropicResolution)

# cpack stuff
INCLUDE(CPack)

# set the name of the input image used to run the test
SET(INPUT_IMAGE ${CMAKE_SOURCE_DIR}/images/cthead1.png)

#include some macros from another file...
INCLUDE(IJMacros.txt)

#The following lines are required to use Dart
ENABLE_TESTING()
INCLUDE(Dart)
FIND_PROGRAM(IMAGE_COMPARE ImageCompare)

#Declare any external dependencies that your project may have here.
#examples include: ITK, VTK, JPEG, PNG, OpenGL, ZLIB, Perl, Java
#If you're not sure what name to use, look in the Modules directory of your
#cmake install and check that a file named Find(Package).cmake exists
SET(Required_Packages
  ITK
)

#this foreach loads all of the packages that you specified as required.
#It shouldn't need to be modified.
FOREACH(Package ${Required_Packages})
  LOADPACKAGE(${Package})
ENDFOREACH(Package)

#Set any libraries that your project depends on.
#examples: ITKCommon, VTKRendering, etc
SET(Libraries
  ITKCommon
  ITKBasicFilters
  ITKIO
)


# install devel files
FILE(GLOB develFiles *.h *.txx) 
FOREACH(f ${develFiles})
  INSTALL_FILES(/include/InsightToolkit/BasicFilters FILES ${f})
ENDFOREACH(f)



# option for wrapping
OPTION(BUILD_WRAPPERS "Wrap library" OFF)
IF(BUILD_WRAPPERS)
   SUBDIRS(Wrapping)
ENDIF(BUILD_WRAPPERS)
   
   

#the following block of code is an example of how to build an executable in
#cmake.  Unmodified, it will add an executable called "MyExe" to the project.
#MyExe will be built using the files MyClass.h and MyClass.cxx, and it will
#be linked to all the libraries you specified above. 
#You can build more than one executable per project

SET(CurrentExe "check")
ADD_EXECUTABLE(${CurrentExe} ${CurrentExe}.cxx)
TARGET_LINK_LIBRARIES(${CurrentExe} ${Libraries})

SET(CurrentExe "check3D")
ADD_EXECUTABLE(${CurrentExe} ${CurrentExe}.cxx)
TARGET_LINK_LIBRARIES(${CurrentExe} ${Libraries})

#the following line is an example of how to add a test to your project.
#Testname is the title for this particular test.  ExecutableToRun is the
#program which will be running this test.  It can either be a part of this
#project or an external executable.  After that list any args that are needed
#for this test.  Include as many tests as you like.  If your project doesn't have
#any tests you can comment out or delete the following line.
# ADD_TEST(Testname ExecutableToRun arg1 arg2 arg3)


ADD_TEST(Run_0.5_0 check 0.5 0 ${CMAKE_SOURCE_DIR}/images/cthead1.png cthead1_0.5_0.img)
ADD_TEST(CompareImage_0.5_0 ${IMAGE_COMPARE} cthead1_0.5_0.img ${CMAKE_SOURCE_DIR}/images/cthead1_0.5_0.img)

ADD_TEST(Run_1_0 check 1 0 ${CMAKE_SOURCE_DIR}/images/cthead1.png cthead1_1_0.img)
ADD_TEST(CompareImage_1_0 ${IMAGE_COMPARE} cthead1_1_0.img ${CMAKE_SOURCE_DIR}/images/cthead1_1_0.img)

ADD_TEST(Run_2_0 check 2 0 ${CMAKE_SOURCE_DIR}/images/cthead1.png cthead1_2_0.img)
ADD_TEST(CompareImage_2_0 ${IMAGE_COMPARE} cthead1_2_0.img ${CMAKE_SOURCE_DIR}/images/cthead1_2_0.img)

ADD_TEST(Run_0_0 check 0 0 ${CMAKE_SOURCE_DIR}/images/cthead1.png cthead1_0_0.img)
ADD_TEST(CompareImage_0_0 ${IMAGE_COMPARE} cthead1_0_0.img ${CMAKE_SOURCE_DIR}/images/cthead1_0_0.img)


ADD_TEST(Run_0.5_1 check 0.5 1 ${CMAKE_SOURCE_DIR}/images/cthead1.png cthead1_0.5_1.img)
ADD_TEST(CompareImage_0.5_1 ${IMAGE_COMPARE} cthead1_0.5_1.img ${CMAKE_SOURCE_DIR}/images/cthead1_0.5_1.img)

ADD_TEST(Run_1_1 check 1 1 ${CMAKE_SOURCE_DIR}/images/cthead1.png cthead1_1_1.img)
ADD_TEST(CompareImage_1_1 ${IMAGE_COMPARE} cthead1_1_1.img ${CMAKE_SOURCE_DIR}/images/cthead1_1_1.img)

ADD_TEST(Run_2_1 check 2 1 ${CMAKE_SOURCE_DIR}/images/cthead1.png cthead1_2_1.img)
ADD_TEST(CompareImage_2_1 ${IMAGE_COMPARE} cthead1_2_1.img ${CMAKE_SOURCE_DIR}/images/cthead1_2_1.img)

ADD_TEST(Run_0_1 check 0 1 ${CMAKE_SOURCE_DIR}/images/cthead1.png cthead1_0_1.img)
ADD_TEST(CompareImage_0_1 ${IMAGE_COMPARE} cthead1_0_1.img ${CMAKE_SOURCE_DIR}/images/cthead1_0_1.img)




ADD_TEST(Run3D_0.5_0 check3D 0.5 0 ${CMAKE_SOURCE_DIR}/images/ESCells.img ESCells_0.5_0.img)
ADD_TEST(CompareImage3D_0.5_0 ${IMAGE_COMPARE} ESCells_0.5_0.img ${CMAKE_SOURCE_DIR}/images/ESCells_0.5_0.img)

ADD_TEST(Run3D_1_0 check3D 1 0 ${CMAKE_SOURCE_DIR}/images/ESCells.img ESCells_1_0.img)
ADD_TEST(CompareImage3D_1_0 ${IMAGE_COMPARE} ESCells_1_0.img ${CMAKE_SOURCE_DIR}/images/ESCells_1_0.img)

ADD_TEST(Run3D_2_0 check3D 2 0 ${CMAKE_SOURCE_DIR}/images/ESCells.img ESCells_2_0.img)
ADD_TEST(CompareImage3D_2_0 ${IMAGE_COMPARE} ESCells_2_0.img ${CMAKE_SOURCE_DIR}/images/ESCells_2_0.img)

ADD_TEST(Run3D_0_0 check3D 0 0 ${CMAKE_SOURCE_DIR}/images/ESCells.img ESCells_0_0.img)
ADD_TEST(CompareImage3D_0_0 ${IMAGE_COMPARE} ESCells_0_0.img ${CMAKE_SOURCE_DIR}/images/ESCells_0_0.img)


ADD_TEST(Run3D_0.5_1 check3D 0.5 1 ${CMAKE_SOURCE_DIR}/images/ESCells.img ESCells_0.5_1.img)
ADD_TEST(CompareImage3D_0.5_1 ${IMAGE_COMPARE} ESCells_0.5_1.img ${CMAKE_SOURCE_DIR}/images/ESCells_0.5_1.img)

ADD_TEST(Run3D_1_1 check3D 1 1 ${CMAKE_SOURCE_DIR}/images/ESCells.img ESCells_1_1.img)
ADD_TEST(CompareImage3D_1_1 ${IMAGE_COMPARE} ESCells_1_1.img ${CMAKE_SOURCE_DIR}/images/ESCells_1_1.img)

ADD_TEST(Run3D_2_1 check3D 2 1 ${CMAKE_SOURCE_DIR}/images/ESCells.img ESCells_2_1.img)
ADD_TEST(CompareImage3D_2_1 ${IMAGE_COMPARE} ESCells_2_1.img ${CMAKE_SOURCE_DIR}/images/ESCells_2_1.img)

ADD_TEST(Run3D_0_1 check3D 0 1 ${CMAKE_SOURCE_DIR}/images/ESCells.img ESCells_0_1.img)
ADD_TEST(CompareImage3D_0_1 ${IMAGE_COMPARE} ESCells_0_1.img ${CMAKE_SOURCE_DIR}/images/ESCells_0_1.img)

