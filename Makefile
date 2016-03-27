TARGET		?= MOTO_F3

ifeq ($(TARGET),SPARKY2)
BUILDDIR = build_sparky2
include source/targets/sparky2/Makefile
endif
ifeq ($(TARGET),MOTO_F3)
BUILDDIR = build_motof3
include source/targets/MotoF3/Makefile
endif
