#pragma once
#include "ui/Widgets.h"

#include <string>
#include <vector>

namespace ui {

// ---------------------------------------------------------------------------
// A file browser, in the sense that it browses files: it walks directories,
// shows what is in them with sizes and dates, and opens, saves or deletes.
//
// It exists because designs are files now. A dropdown of names that happens to
// map to one hard-coded folder is fine until you want to keep engines in
// folders, copy one in from somewhere else, or look at what you have - and then
// it is not a browser at all, it is a list.
// ---------------------------------------------------------------------------
class FileBrowser {
public:
    enum class Action { None, Open, Save };

    struct Result {
        Action      action = Action::None;
        std::string path;
    };

    void open(const std::string& startDir, const std::string& suggestedName);
    void close() { m_visible = false; }
    bool visible() const { return m_visible; }

    // Draws the panel and returns what the person asked for, if anything.
    Result draw(Ui& ui, float x, float y, float w, float h);

private:
    struct Entry {
        std::string name;
        bool        directory = false;
        std::uintmax_t size = 0;
        std::string modified;
        std::string title;      // the engine's own name, read out of the file
    };

    void rescan();
    void navigate(const std::string& into);

    bool        m_visible = false;
    std::string m_dir = "designs";
    std::string m_filename;
    std::string m_message;
    std::vector<Entry> m_entries;
    int   m_selected = -1;
    float m_scroll = 0.0f;
    // Deleting is the one thing here that cannot be undone, so it takes two
    // presses and says which file it means in between.
    std::string m_pendingDelete;
};

} // namespace ui
