#
# Native build (GLFW + OpenGL 3), based on the Dear ImGui example_glfw_opengl3 Makefile.
#
# You will need GLFW (http://www.glfw.org):
#   Linux:     apt-get install libglfw3-dev
#   macOS:     brew install glfw
#   MSYS2:     pacman -S --noconfirm --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-glfw
#
# Targets:
#   make          build ./fourbar_gui
#   make test     build and run the kinematics unit tests (no GUI libraries needed)
#   make clean
#

EXE = fourbar_gui
BUILD_DIR = build
IMGUI_DIR = third_party/imgui
IMPLOT_DIR = third_party/implot

APP_SOURCES = src/main.cpp src/app.cpp src/linkage.cpp
IMGUI_SOURCES = $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_draw.cpp $(IMGUI_DIR)/imgui_tables.cpp \
                $(IMGUI_DIR)/imgui_widgets.cpp $(IMGUI_DIR)/imgui_demo.cpp \
                $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp
IMPLOT_SOURCES = $(IMPLOT_DIR)/implot.cpp $(IMPLOT_DIR)/implot_items.cpp
SOURCES = $(APP_SOURCES) $(IMGUI_SOURCES) $(IMPLOT_SOURCES)
OBJS = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(basename $(notdir $(SOURCES)))))
vpath %.cpp src $(IMGUI_DIR) $(IMGUI_DIR)/backends $(IMPLOT_DIR)

UNAME_S := $(shell uname -s)
LINUX_GL_LIBS = -lGL

CXXFLAGS = -std=c++17 -Isrc -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends -I$(IMPLOT_DIR)
CXXFLAGS += -O2 -Wall -Wformat
LIBS =

##---------------------------------------------------------------------
## BUILD FLAGS PER PLATFORM
##---------------------------------------------------------------------

ifeq ($(UNAME_S), Linux) #LINUX
	ECHO_MESSAGE = "Linux"
	LIBS += $(LINUX_GL_LIBS) `pkg-config --static --libs glfw3`
	CXXFLAGS += `pkg-config --cflags glfw3`
endif

ifeq ($(UNAME_S), Darwin) #APPLE
	ECHO_MESSAGE = "Mac OS X"
	LIBS += -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
	LIBS += -L/usr/local/lib -L/opt/local/lib -L/opt/homebrew/lib
	LIBS += -lglfw
	CXXFLAGS += -I/usr/local/include -I/opt/local/include -I/opt/homebrew/include
endif

ifeq ($(OS), Windows_NT)
	ECHO_MESSAGE = "MinGW"
	LIBS += -lglfw3 -lgdi32 -lopengl32 -limm32
	CXXFLAGS += `pkg-config --cflags glfw3`
endif

##---------------------------------------------------------------------
## BUILD RULES
##---------------------------------------------------------------------

all: $(EXE)
	@echo Build complete for $(ECHO_MESSAGE)

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(EXE): $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

# Unit tests only need the kinematics core
TEST_EXE = $(BUILD_DIR)/test_linkage
$(TEST_EXE): tests/test_linkage.cpp src/linkage.cpp src/linkage.h | $(BUILD_DIR)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Isrc -o $@ tests/test_linkage.cpp src/linkage.cpp

test: $(TEST_EXE)
	./$(TEST_EXE)

clean:
	rm -rf $(BUILD_DIR) $(EXE)

.PHONY: all test clean
