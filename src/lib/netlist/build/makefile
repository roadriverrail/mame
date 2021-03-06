###########################################################################
#
#   makefile
#
#   Core makefile for building netlist
#
###########################################################################

#
# Notes:
#    These should go into some compile.md
#
#    Mingw notes:
#
#		graphviz is needed for documentation
#      		pacman -S mingw-w64-x86_64-graphviz
#
#       Please build documentation using
#           make mingw PARAMS=doc
#
#

.DEFAULT_GOAL := all

SRC = ..
PSRC = $(SRC)/plib
VSBUILD = $(SRC)/buildVS
DOC = $(SRC)/documentation

TIDY_FLAGSX = -checks=*,-google*,-hicpp*,readability*,-fuchsia*,cert-*,-android-*,
TIDY_FLAGSX += -llvm-header-guard,
# TIDY_FLAGSX += -cppcoreguidelines-pro-type-reinterpret-cast,
TIDY_FLAGSX += -cppcoreguidelines-pro-bounds-pointer-arithmetic,
#TIDY_FLAGSX += -cppcoreguidelines-owning-memory,
TIDY_FLAGSX += -modernize-use-default-member-init,-cppcoreguidelines-pro-bounds-constant-array-index,
TIDY_FLAGSX += -modernize-pass-by-value,
#TIDY_FLAGSX += -cppcoreguidelines-pro-type-static-cast-downcast,
TIDY_FLAGSX += -cppcoreguidelines-avoid-magic-numbers,
TIDY_FLAGSX += -cppcoreguidelines-macro-usage,
TIDY_FLAGSX += -cppcoreguidelines-non-private-member-variables-in-classes,-misc-non-private-member-variables-in-classes,
TIDY_FLAGSX += -bugprone-macro-parentheses,
#TIDY_FLAGSX += -misc-macro-parentheses,
TIDY_FLAGSX += -bugprone-too-small-loop-variable,
TIDY_FLAGSX += -modernize-use-trailing-return-type,
#TIDY_FLAGSX += -cppcoreguidelines-pro-bounds-array-to-pointer-decay,
TIDY_FLAGSX += -readability-magic-numbers,-readability-braces-around-statements,
TIDY_FLAGSX += -readability-implicit-bool-conversion,
TIDY_FLAGSX += -readability-named-parameter,-readability-function-size,
TIDY_FLAGSX += -llvmlibc-restrict-system-libc-headers,-llvmlibc-implementation-in-namespace,-llvmlibc-callee-namespace,
#TIDY_FLAGSX += -cppcoreguidelines-avoid-non-const-global-variables

space :=
space +=
TIDY_FLAGS = -p $(OBJ) $(subst $(space),,$(TIDY_FLAGSX))
TIDY_SOURCES = $(ALLSOURCES)

#TIDY_SOURCES = $(SRC)/devices/nld_7442.cpp $(SRC)/devices/nld_7492.cpp

TIDY_FLAGS = -p $(OBJ) -checks=llvm-include-order,llvm-namespace-comment,modernize-use-override,modernize-use-using -fix
#TIDY_FLAGS = -checks=llvm-include-order -fix
#TIDY_FLAGS = -checks=llvm-namespace-comment -fix
#TIDY_FLAGS = -checks=modernize-use-override -fix
#TIDY_FLAGS = -checks=modernize-use-using -fix

ifeq ($(subst Windows_NT,windows,$(OS)),windows)
OBJ = obj/mingw
EXESUFFIX = .exe
DOXYGEN = doxygen.exe
HOSTARCH = WINDOWS
CEXTRAFLAGS = -DUNICODE -D_UNICODE -D_WIN32_WINNT=0x0501 -DWIN32_LEAN_AND_MEAN
LDEXTRAFLAGS = -Wl,--subsystem,console -municode
LIBS = $(EXTRALIBS)
MD = @mkdir.exe
RM = @rm.exe
SHELL = sh.exe
else
OBJ = obj/nix
EXESUFFIX :=
DOXYGEN = @./doxygen
HOSTARCH = NIX
LIBS = -lpthread -ldl $(EXTRALIBS)
MD = @mkdir
RM = @rm
endif

PYTHON=@python

TIDY_DB = $(OBJ)/compile_commands.json


#LTO decreases performance :-(
#LTO = -flto=4  -fuse-linker-plugin -Wodr

CCOREFLAGS = -g -O3 -std=c++14 -I$(CURDIR)/.. -I$(CURDIR)/../..

CFLAGS =  $(LTO) $(CCOREFLAGS) $(CEXTRAFLAGS)
LDFLAGS = $(LTO) -g -O3 -std=c++14 $(LDEXTRAFLAGS)

CC = g++
LD = g++
CLANG_TIDY = clang-tidy-11
DEPENDCC=$(CC)


ifndef FAST
FAST=0
endif

ifeq ($(FAST),1)
CFLAGS += -DNL_USE_ACADEMIC_SOLVERS=0
endif

TARGETS = nltool$(EXESUFFIX) nlwav$(EXESUFFIX)

NLOBJ = $(OBJ)
POBJ = $(OBJ)/plib
TESTOBJ = $(OBJ)/tests

DEPEND := $(OBJ)/.depend

OBJDIRS = $(OBJ) \
			$(OBJ)/analog \
			$(OBJ)/solver \
			$(OBJ)/devices \
			$(OBJ)/plib \
			$(OBJ)/devices \
			$(OBJ)/macro \
			$(OBJ)/tests \
			$(OBJ)/tools \
			$(OBJ)/prg \
			$(OBJ)/generated \

DEVSOURCES = $(wildcard $(SRC)/devices/*.cpp)
DEVSOURCES += $(wildcard $(SRC)/analog/*.cpp)
DEVSOURCES += $(wildcard $(SRC)/macro/*.cpp)
DEVSOURCES += $(SRC)/solver/nld_solver.cpp

TESTSOURCES = $(wildcard $(SRC)/tests/*.cpp)

CORESOURCES := \
	$(SRC)/solver/nld_matrix_solver.cpp \
	$(SRC)/nl_base.cpp \
	$(SRC)/nl_parser.cpp \
	$(SRC)/nl_setup.cpp \
	$(SRC)/nl_factory.cpp \
	$(SRC)/tools/nl_convert.cpp \
	$(SRC)/generated/static_solvers.cpp \

MAINSOURCES = $(SRC)/prg/nltool.cpp $(SRC)/prg/nlwav.cpp

PSOURCES := \
	$(PSRC)/pstring.cpp \
	$(PSRC)/pdynlib.cpp \
	$(PSRC)/pexception.cpp \
	$(PSRC)/pfunction.cpp \
	$(PSRC)/pfmtlog.cpp \
	$(PSRC)/pmain.cpp \
	$(PSRC)/poptions.cpp \
	$(PSRC)/ppreprocessor.cpp \
	$(PSRC)/ptokenizer.cpp \
	$(PSRC)/putil.cpp \

NLOBJS = 	$(patsubst $(SRC)%, $(OBJ)%, $(CORESOURCES:.cpp=.o))
NLDEVOBJS = $(patsubst $(SRC)%, $(OBJ)%, $(DEVSOURCES:.cpp=.o))
TESTOBJS = 	$(patsubst $(SRC)%, $(OBJ)%, $(TESTSOURCES:.cpp=.o))
MAINOBJS = 	$(patsubst $(SRC)%, $(OBJ)%, $(MAINSOURCES:.cpp=.o))
POBJS = 	$(patsubst $(SRC)%, $(OBJ)%, $(PSOURCES:.cpp=.o))

OBJS = $(POBJS) $(NLOBJS) $(NLDEVOBJS) $(TESTOBJS)

VSBUILDS = \
	$(VSBUILD/netlistlib.vcxproj) \
	$(VSBUILD/netlistlib.vcxproj.user \
	$(VSBUILD/nltool.vcxproj \
	$(VSBUILD/netlistlib.vcxproj.filters  \
	$(VSBUILD/nltool.vcxproj.filters \
	$(VSBUILD/netlist.sln \

DOCS = \
	doxygen.conf \
	$(DOC)/doc.css \
	$(DOC)/mainpage.dox.h \
	$(DOC)/primer_1.dox.h \
	$(DOC)/structure.dox.h \
	$(DOC)/test1-50r.svg \

ALL_OBJS = $(OBJS) $(MAINOBJS)

ALL_TIDY_FILES = $(ALL_OBJS:.o=.json)
ALLSOURCES = $(DEVSOURCES) $(CORESOURCES) $(TESTSOURCES) $(MAINSOURCES) $(PSOURCES)
ALLFILES = $(ALLSOURCES) $(VSBUILDS) $(DOCS)

MAKEFILE_TARGETS_WITHOUT_INCLUDE := \
	clang clang-5 clang-libc gcc9 mingw native nvcc \
	clean depend doc generated manpages maketree \
	runtests tidy

BUILD_DIRS = $(OBJDIRS) man html

# git archive HEAD --prefix=project-name-version/ \
#     --format=zip -o project-name-version.zip

#-------------------------------------------------
# PHONY
#-------------------------------------------------

.PHONY: $(MAKEFILE_TARGETS_WITHOUT_INCLUDE)

#-------------------------------------------------
# all
#-------------------------------------------------

all: maketree generated $(DEPEND) $(TARGETS)

#-------------------------------------------------
# clean
#-------------------------------------------------

clean:
	$(RM) -rf $(OBJS) $(TARGETS) $(MAINOBJS) $(DEPEND) doxy/*

#-------------------------------------------------
# nltool
#-------------------------------------------------

nltool$(EXESUFFIX): $(OBJ)/prg/nltool.o $(OBJS)
	@echo Linking $@...
	$(LD) -o $@ $(LDFLAGS) $^ $(LIBS)

nlwav$(EXESUFFIX): $(OBJ)/prg/nlwav.o $(OBJS)
	@echo Linking $@...
	$(LD) -o $@ $(LDFLAGS) $^ $(LIBS)

#-------------------------------------------------
# directories
#-------------------------------------------------

$(sort $(BUILD_DIRS)):
	$(MD) -p $@

maketree: $(sort $(BUILD_DIRS))

#-------------------------------------------------
# Special targets
#-------------------------------------------------

native:
	$(MAKE) CEXTRAFLAGS="-march=native -msse4.2 -Wall -Wpedantic -Wsign-compare -Wextra "

gcc9:
	$(MAKE) CC=g++-9 LD=g++-9 CEXTRAFLAGS="-march=native -Wall -pedantic -Wpedantic -fext-numeric-literals -Wsign-compare -Wextra" EXTRALIBS="-lquadmath"

clang:
	#$(MAKE) CC=clang++-11 LD=clang++-11 OBJ=obj/clang CEXTRAFLAGS="-march=native -msse4.2 -Weverything -Wall -pedantic -Wpedantic -Wunused-private-field -Wno-padded -Wno-unused-template -Wno-missing-variable-declarations -Wno-float-equal -Wconversion -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-format-nonliteral  -Wno-exit-time-destructors"
	$(MAKE) CC=clang++-11 LD=clang++-11 OBJ=obj/clang CEXTRAFLAGS="-march=native \
		-mllvm -inline-threshold=2000 \
		-Wunknown-warning-option \
		-Weverything -Wall -pedantic -Wpedantic -Wunused-private-field  \
		-Werror -Wno-padded -Wno-weak-vtables -Wno-weak-template-vtables -Wunused-template \
		-Wmissing-variable-declarations -Wno-float-equal -Wconversion \
		-Wno-c++98-compat -Wno-c++98-compat-pedantic -Wformat-nonliteral \
		-Wno-exit-time-destructors -Winconsistent-missing-destructor-override \
		-Wno-return-std-move-in-c++11 -Wno-unreachable-code \
		-Wno-missing-prototypes"

clang-libc:
	#clang-11 currently broken
	#$(MAKE) CC=clang++-11 LD=clang++-11 OBJ=obj/clang CEXTRAFLAGS="-march=native
	$(MAKE) CC=clang++-10 LD=clang++-10 OBJ=obj/clang-libc CEXTRAFLAGS="-march=native \
		-stdlib=libc++ -mllvm -inline-threshold=2000 \
		-Wunknown-warning-option \
		-Weverything -Wall -pedantic -Wpedantic -Wunused-private-field  \
		-Werror -Wno-padded -Wno-weak-vtables -Wno-weak-template-vtables -Wunused-template \
		-Wmissing-variable-declarations -Wno-float-equal -Wconversion \
		-Wno-c++98-compat -Wno-c++98-compat-pedantic -Wformat-nonliteral \
		-Wno-exit-time-destructors -Winconsistent-missing-destructor-override \
		-Wno-return-std-move-in-c++11 -Wno-unreachable-code" \
		LDEXTRAFLAGS=-stdlib=libc++

clang-5:
	$(MAKE) CC=clang++-5.0 LD=clang++-5.0 CEXTRAFLAGS="-march=native -Weverything -Werror -Wno-inconsistent-missing-destructor-override -Wno-unreachable-code -Wno-padded -Wno-weak-vtables -Wno-missing-variable-declarations -Wconversion -Wno-c++98-compat -Wno-float-equal -Wno-global-constructors -Wno-c++98-compat-pedantic -Wno-format-nonliteral -Wno-weak-template-vtables -Wno-exit-time-destructors"

nvcc:
	$(MAKE) CC=/usr/local/cuda-10.2/bin/nvcc LD=/usr/local/cuda-10.2/bin/nvcc \
		OBJ=obj/nvcc CEXTRAFLAGS="--std c++14 -x cu  -DNVCCBUILD=102 --expt-extended-lambda \
		--expt-relaxed-constexpr --default-stream per-thread --restrict \
		--ftemplate-depth 1024  \
		-Xcompiler -O6 -Xcompiler -march=native -ccbin g++-8 " \
		DEPENDCC=g++

mingw:
	$(MAKE) CEXTRAFLAGS="-DUNICODE -D_UNICODE -D_WIN32_WINNT=0x0501 \
		-DWIN32_LEAN_AND_MEAN" LDEXTRAFLAGS="-Wl,--subsystem,console \
		-municode" LIBS= MD=@mkdir.exe SHELL=sh.exe DOXYGEN=doxygen.exe $(PARAMS)

#-------------------------------------------------
# regression tests
#-------------------------------------------------

runtests: $(TARGETS)
	./nltool$(EXESUFFIX) -c tests

#-------------------------------------------------
# man pages
#-------------------------------------------------

manpages: maketree $(TARGETS) man/nltool.1 man/nlwav.1 html/nltool.html html/nlwav.html

#-------------------------------------------------
# documentation
#-------------------------------------------------

doc:
	./nltool$(EXESUFFIX) -c docheader > ../documentation/devsyn.dox.h
	$(DOXYGEN) doxygen.conf

#-------------------------------------------------
# generated
#-------------------------------------------------

../generated/lib_entries.hxx: $(DEVSOURCES)
	@echo creating $@
	$(PYTHON) create_lib_entries.py $^ > $@

../generated/nld_devinc.h: $(DEVSOURCES)
	@echo creating $@
	$(PYTHON) create_devinc.py $^ > $@

.PHONY: generated
generated: ../generated/lib_entries.hxx ../generated/nld_devinc.h

#-------------------------------------------------
# depends
#-------------------------------------------------

.PHONY: depend
depend: $(DEPEND)

$(DEPEND): $(OBJ) | $(ALLSOURCES)
	@echo creating $(DEPEND)
	@$(RM) -f $(DEPEND)
	@for i in $(ALLSOURCES); do \
		$(DEPENDCC) $(CCOREFLAGS) -MM $$i -MT `echo $$i | sed -e 's+$(SRC)+$(OBJ)+' -e 's+.cpp+.o+' ` >> $(DEPEND); \
	done

# Include only if the goal needs it
ifeq ($(filter $(MAKECMDGOALS),$(MAKEFILE_TARGETS_WITHOUT_INCLUDE)),)
-include $(DEPEND)
endif

#-------------------------------------------------
# clang tidy
#-------------------------------------------------

tidy_db: compile_commands_prefix $(ALL_TIDY_FILES) compile_commands_postfix

tidy: tidy_db
	@echo running tidy
	@for i in $(TIDY_SOURCES); do \
		$(CLANG_TIDY) $$i $(TIDY_FLAGS) -header-filter=.*; \
	done

#-------------------------------------------------
# generic rules
#-------------------------------------------------

$(OBJ)/%.o: $(SRC)/%.cpp
	@echo Compiling $<...
	@$(CC) $(CDEFS) $(CFLAGS) -c $< -o $@

$(OBJ)/%.pp: $(SRC)/%.cpp
	@echo Compiling $<...
	@$(CC) $(CDEFS) $(CFLAGS) -E $< -o $@

$(OBJ)/%.s: $(SRC)/%.cpp
	@echo Compiling $<...
	@$(CC) $(CDEFS) $(CFLAGS) -S $< -o $@

$(OBJ)/%.a:
	@echo Archiving $@...
	$(RM) $@
	$(AR) $(ARFLAGS) $@ $^

$(OBJ)/%.json: $(SRC)/%.cpp
	@echo Building compile database entry for $< ...
	@echo { \"directory\": \".\", >> $(TIDY_DB)
	@echo   \"command\": \"$(CC) $(CDEFS) $(CFLAGS) -c $< -o dummy.o\", >> $(TIDY_DB)
	@echo   \"file\": \"$(CURDIR)/$<\" } >> $(TIDY_DB)
	@echo "," >> $(TIDY_DB)

man/%.1: %$(EXESUFFIX)
	@echo Building $@...
	@help2man --include=$*.help2man --no-info ./$< > $@

html/%.html: man/%.1
	@echo Building $@...
	@groff -mandoc -Thtml $< > $@

compile_commands_prefix:
	@echo "[" > $(TIDY_DB)

compile_commands_postfix:
	@echo "]" >> $(TIDY_DB)
