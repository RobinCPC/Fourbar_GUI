// ImGui front end for the four-bar and eight-bar linkage analysis.
#pragma once

#define FOURBAR_APP_VERSION "1.0.0"

namespace app {

// Called once after ImGui/ImPlot contexts are created.
void Init();

// Draws the whole UI for one frame. `dt` is the frame time in seconds.
void RenderUI(float dt);

}  // namespace app
