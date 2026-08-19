#include "ui/FileBrowser.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

namespace ui {
namespace fs = std::filesystem;
namespace {

std::string humanSize(std::uintmax_t bytes) {
    char buf[32];
    if (bytes < 1024) std::snprintf(buf, sizeof(buf), "%llu B",
                                    static_cast<unsigned long long>(bytes));
    else if (bytes < 1024 * 1024) std::snprintf(buf, sizeof(buf), "%.1f kB", bytes / 1024.0);
    else std::snprintf(buf, sizeof(buf), "%.1f MB", bytes / (1024.0 * 1024.0));
    return buf;
}

std::string modifiedString(const fs::path& p) {
    std::error_code ec;
    const auto t = fs::last_write_time(p, ec);
    if (ec) return "";
    // file_clock to system_clock is only spelled portably in C++20 via
    // clock_cast, which MSVC has.
    const auto sys = std::chrono::clock_cast<std::chrono::system_clock>(t);
    const std::time_t tt = std::chrono::system_clock::to_time_t(sys);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &tm);
    return buf;
}

// The engine's own title, read out of the file. A filename is what somebody
// typed; this is what the engine calls itself, and it is the thing worth
// showing when you are picking between them.
std::string titleOf(const fs::path& p) {
    std::ifstream in(p);
    if (!in) return {};
    std::string line;
    int scanned = 0;
    while (std::getline(in, line) && ++scanned < 40) {
        const std::size_t k = line.find("\"name\"");
        if (k == std::string::npos) continue;
        const std::size_t a = line.find('"', line.find(':', k) + 1);
        const std::size_t b = a == std::string::npos ? a : line.find('"', a + 1);
        if (a == std::string::npos || b == std::string::npos) break;
        return line.substr(a + 1, b - a - 1);
    }
    return {};
}

bool interesting(const fs::path& p) {
    const std::string ext = p.extension().string();
    return ext == ".json" || ext == ".eng";
}

} // namespace

void FileBrowser::open(const std::string& startDir, const std::string& suggestedName) {
    m_visible = true;
    m_message.clear();
    m_pendingDelete.clear();
    std::error_code ec;
    if (!fs::is_directory(startDir, ec)) fs::create_directories(startDir, ec);
    m_dir = startDir;
    m_filename = suggestedName;
    m_selected = -1;
    m_scroll = 0.0f;
    rescan();
}

void FileBrowser::rescan() {
    m_entries.clear();
    std::error_code ec;
    const fs::path here(m_dir);
    if (here.has_parent_path() || here != here.root_path())
        m_entries.push_back({"..", true, 0, "", ""});

    std::vector<Entry> dirs, files;
    for (const auto& e : fs::directory_iterator(here, ec)) {
        const fs::path& p = e.path();
        if (e.is_directory(ec)) {
            dirs.push_back({p.filename().string(), true, 0, modifiedString(p), ""});
        } else if (e.is_regular_file(ec) && interesting(p)) {
            files.push_back({p.filename().string(), false, e.file_size(ec),
                             modifiedString(p), titleOf(p)});
        }
    }
    // Folders first, then files, each alphabetically - the order every file
    // manager uses because it is the order people look in.
    const auto byName = [](const Entry& a, const Entry& b) { return a.name < b.name; };
    std::sort(dirs.begin(), dirs.end(), byName);
    std::sort(files.begin(), files.end(), byName);
    m_entries.insert(m_entries.end(), dirs.begin(), dirs.end());
    m_entries.insert(m_entries.end(), files.begin(), files.end());
    m_selected = -1;
}

void FileBrowser::navigate(const std::string& into) {
    std::error_code ec;
    fs::path next = into == ".." ? fs::path(m_dir).parent_path() : fs::path(m_dir) / into;
    if (next.empty()) next = fs::current_path(ec);
    if (!fs::is_directory(next, ec)) return;
    m_dir = next.lexically_normal().string();
    m_scroll = 0.0f;
    m_pendingDelete.clear();
    rescan();
}

FileBrowser::Result FileBrowser::draw(Ui& ui, float x, float y, float w, float h) {
    Result result;
    if (!m_visible) return result;

    const Palette& pal = ui.pal();
    ui.rect(x - 6.0f, y - 6.0f, w + 12.0f, h + 12.0f, sf::Color(0, 0, 0, 150));
    ui.rect(x, y, w, h, pal.panel, pal.line, 1.0f);

    ui.text("ENGINE FILES", x + 14.0f, y + 12.0f, 15, pal.text);
    ui.text(m_dir, x + 14.0f, y + 32.0f, 12, pal.dim);
    if (ui.button("CLOSE", x + w - 78.0f, y + 10.0f, 64.0f, 24.0f)) {
        m_visible = false;
        return result;
    }
    if (ui.button("REFRESH", x + w - 158.0f, y + 10.0f, 72.0f, 24.0f)) rescan();

    // ---- Listing -----------------------------------------------------------
    const float listTop = y + 54.0f;
    const float listH   = h - 54.0f - 82.0f;
    const float rowH    = 22.0f;
    ui.rect(x + 10.0f, listTop, w - 20.0f, listH, sf::Color(0, 0, 0, 60), pal.line, 1.0f);

    const sf::FloatRect area({x + 10.0f, listTop}, {w - 20.0f, listH});
    ui.beginScroll(area, m_scroll, static_cast<float>(m_entries.size()) * rowH + 6.0f);
    for (std::size_t i = 0; i < m_entries.size(); ++i) {
        const Entry& e = m_entries[i];
        const float ry = listTop + 3.0f + rowH * static_cast<float>(i) - m_scroll;
        const bool selected = static_cast<int>(i) == m_selected;

        if (ui.button(std::string(), x + 12.0f, ry, w - 24.0f, rowH - 2.0f, selected,
                      selected ? pal.accent : sf::Color::Transparent)) {
            if (e.directory) {
                navigate(e.name);
                ui.endScroll();
                return result;
            }
            m_selected = static_cast<int>(i);
            m_filename = e.name;
            m_pendingDelete.clear();
        }
        const sf::Color nameCol = e.directory ? pal.intake : (selected ? pal.text : pal.text);
        ui.text((e.directory ? "[ " + e.name + " ]" : e.name), x + 20.0f, ry + 4.0f, 13, nameCol);
        if (!e.directory) {
            // The title sits beside the filename, dimmed, so the list reads as
            // engines rather than as paths.
            if (!e.title.empty())
                ui.text(e.title, x + 20.0f + ui.textWidth(e.name, 13) + 14.0f, ry + 5.0f,
                        12, selected ? pal.text : pal.dim);
            ui.right(humanSize(e.size), x + w - 150.0f, ry + 4.0f, 12, pal.dim);
        }
        ui.text(e.modified, x + w - 140.0f, ry + 4.0f, 12, pal.dim);
    }
    ui.endScroll();

    // ---- Filename and actions ----------------------------------------------
    const float footY = y + h - 74.0f;
    ui.column(x + 14.0f, footY, w - 200.0f);
    ui.textField("FILE NAME", m_filename, 64);

    const float bx = x + w - 178.0f;
    if (ui.button("OPEN", bx, footY + 4.0f, 78.0f, 26.0f) && !m_filename.empty()) {
        result.action = Action::Open;
        result.path = (fs::path(m_dir) / m_filename).string();
        m_visible = false;
        return result;
    }
    if (ui.button("SAVE", bx + 86.0f, footY + 4.0f, 78.0f, 26.0f) && !m_filename.empty()) {
        std::string name = m_filename;
        if (name.find('.') == std::string::npos) name += ".json";
        result.action = Action::Save;
        result.path = (fs::path(m_dir) / name).string();
        m_visible = false;
        return result;
    }

    // Delete asks twice, and names the file it is about to remove.
    const bool armed = !m_pendingDelete.empty();
    if (ui.button(armed ? "CONFIRM" : "DELETE", bx, footY + 36.0f, 78.0f, 24.0f, armed,
                  armed ? sf::Color(200, 70, 60) : sf::Color::Transparent)) {
        if (armed) {
            std::error_code ec;
            const bool ok = fs::remove(fs::path(m_dir) / m_pendingDelete, ec);
            m_message = ok ? "Deleted " + m_pendingDelete : "Could not delete " + m_pendingDelete;
            m_pendingDelete.clear();
            rescan();
        } else if (m_selected >= 0 &&
                   !m_entries[static_cast<std::size_t>(m_selected)].directory) {
            m_pendingDelete = m_entries[static_cast<std::size_t>(m_selected)].name;
            m_message = "Delete " + m_pendingDelete + "? Press again.";
        } else {
            m_message = "Select a file first";
        }
    }
    if (ui.button("NEW FOLDER", bx + 86.0f, footY + 36.0f, 78.0f, 24.0f)) {
        std::error_code ec;
        const std::string folder = m_filename.empty() ? std::string("new folder") : m_filename;
        if (fs::create_directory(fs::path(m_dir) / folder, ec)) {
            m_message = "Created " + folder;
            rescan();
        } else {
            m_message = "Could not create " + folder;
        }
    }

    if (!m_message.empty())
        ui.text(m_message, x + 14.0f, y + h - 20.0f, 12,
                armed ? sf::Color(230, 120, 100) : pal.dim);
    return result;
}

} // namespace ui
