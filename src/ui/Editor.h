#pragma once
#include "input/Gamepad.h"
#include "sim/Dyno.h"
#include "sim/EngineDesign.h"
#include "ui/FileBrowser.h"
#include "ui/Widgets.h"

#include <string>
#include <vector>

namespace ui {

struct EditorResult {
    bool changed = false;   // a control moved this frame
    bool apply   = false;   // build the engine from the design now
    bool revert  = false;   // throw the edits away
};

// ---------------------------------------------------------------------------
// The engine editor: every number the simulation runs on, grouped the way an
// engine is actually specified, with the consequences of each choice shown
// beside it - displacement, overlap, tuned lengths, float speed - and a
// dynamometer to answer the only question the numbers are really for.
// ---------------------------------------------------------------------------
class Editor {
public:
    bool visible   = false;
    bool liveApply = true;
    // Set when bindings were read from disk at startup, so the controls tab can
    // say whether it is showing what was saved or what was guessed.
    bool padLoaded = false;

    // Draws the whole overlay. `dirty` says the design on screen is ahead of
    // the engine that is running, which is what the Apply button lights up for.
    // The gamepad is passed in because its bindings are edited here: they are
    // the person's, not the engine's, which is why they are not part of the
    // design and are saved separately.
    EditorResult draw(Ui& ui, sim::EngineDesign& d, sim::Dyno& dyno,
                      input::Gamepad& pad, bool dirty, float dt);

    void status(const std::string& s, float seconds = 3.0f);

private:
    void tab(Ui& ui, int index, const char* name, float x, float& y);
    void body(Ui& ui, sim::EngineDesign& d, EditorResult& r,
              const sim::DesignSummary& sum, const sim::Dyno& dyno,
              input::Gamepad& pad);
    void controlsTab(Ui& ui, input::Gamepad& pad, float top);
    void sidebar(Ui& ui, const sim::EngineDesign& d, const sim::DesignSummary& sum,
                 sim::Dyno& dyno);
    void firingChart(Ui& ui, const sim::DesignSummary& sum, float x, float y,
                     float w, float h);
    void dynoChart(Ui& ui, sim::Dyno& dyno, float x, float y, float w, float h);
    void mapChart(Ui& ui, sim::Dyno& dyno, float x, float y, float w, float h);

    int   m_tab = 0;
    float m_scroll[12]{};
    float m_contentH[12]{};
    float m_padSaveFlash = 0.0f;
    FileBrowser m_files;
    std::string m_status;
    float m_statusTimer = 0.0f;
};

} // namespace ui
