// ---------------------------------------------------------------------------
// Regression harness.
//
// Almost everything in this simulation is a coupled feedback loop: breathing
// feeds combustion feeds temperature feeds breathing, and the idle controller
// sits on top of all of it. A change that looks local is not - adding the
// runner wave model turned every engine into a supercharger at every speed,
// and nothing about the diff said so. The only way to see that is to run every
// preset and compare against what it did yesterday.
//
// Each preset is dynoed at wide-open throttle and then left to idle on its own.
// Peak power, peak torque and the idle band are compared against the recorded
// baselines in tools/baselines.txt, which is regenerated with --update.
//
//   enginio_validate            check against the baselines, non-zero on failure
//   enginio_validate --update   record what it does now as the new baseline
//   enginio_validate --list     print the measurements and stop
// ---------------------------------------------------------------------------
#include "sim/Engine.h"
#include "sim/EngineDesign.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 1.0 / 44100.0;

// How far a preset may drift before it counts as a change. Peaks are allowed
// more room than idle: the dyno samples a curve at fifteen points, so a peak
// can hop to a neighbouring point on a very small physical change.
constexpr double kPowerTol = 0.04;   // fraction
constexpr double kSpeedTol = 0.10;
constexpr double kIdleTol  = 120.0;  // rpm

struct Measurement {
    double peakKW = 0.0, peakKWrpm = 0.0;
    double peakNm = 0.0, peakNmRpm = 0.0;
    double idleLo = 0.0, idleHi = 0.0, idleMean = 0.0;
    bool   stalled = false;
};

Measurement measure(const sim::EngineDesign& d) {
    Measurement m;
    const sim::EngineParams p = sim::paramsFromDesign(d);

    sim::Engine e(p);
    e.warmUp();
    e.setThrottle(1.0);
    const double lo = std::max(1200.0, d.idleRpm * 1.6);
    for (int k = 0; k <= 14; ++k) {
        const double rpm = lo + (d.redline - lo) * k / 14.0;
        e.setSpeedHold(true, rpm);
        for (int s = 0; s < static_cast<int>(1.2 * 44100); ++s) e.advance(kDt);
        // An uneven-firing engine swings the crank torque hard within a cycle,
        // so one filtered sample is not a measurement.
        double sum = 0.0;
        int n = 0;
        for (int s = 0; s < static_cast<int>(0.6 * 44100); ++s) {
            e.advance(kDt);
            sum += e.heldTorque();
            ++n;
        }
        const double nm = sum / n;
        const double kW = nm * rpm * 2.0 * 3.14159265358979 / 60.0 * 1e-3;
        if (kW > m.peakKW) { m.peakKW = kW; m.peakKWrpm = rpm; }
        if (nm > m.peakNm) { m.peakNm = nm; m.peakNmRpm = rpm; }
    }

    sim::Engine e2(p);
    e2.drivetrain().setParams(sim::drivetrainFromDesign(d));
    e2.setThrottle(0.0);
    e2.setStarter(true);
    for (int s = 0; s < static_cast<int>(1.5 * 44100); ++s) e2.advance(kDt);
    e2.setStarter(false);
    for (int s = 0; s < static_cast<int>(8.0 * 44100); ++s) e2.advance(kDt);
    // Idle is a limit cycle, so its extremes move on floating-point noise that
    // changes nothing physical. The mean over several seconds is the stable
    // measure, and the band is kept only to show alongside it.
    m.idleLo = 1e9;
    double sumRpm = 0.0;
    int nRpm = 0;
    for (int s = 0; s < static_cast<int>(4.0 * 44100); ++s) {
        e2.advance(kDt);
        if (s % 64 == 0) {
            const double r = e2.snapshot().rpm;
            m.idleLo = std::min(m.idleLo, r);
            m.idleHi = std::max(m.idleHi, r);
            sumRpm += r;
            ++nRpm;
        }
    }
    m.idleMean = nRpm ? sumRpm / nRpm : 0.0;
    m.stalled = m.idleMean < 200.0;
    return m;
}

std::string baselinePath(const char* argv0) {
    // Next to the source tree rather than the build directory, so the file is
    // the one under version control.
    std::string exe = argv0 ? argv0 : "";
    const std::size_t cut = exe.find_last_of("/\\");
    std::string dir = cut == std::string::npos ? std::string(".") : exe.substr(0, cut);
    return dir + "/../tools/baselines.txt";
}

std::map<std::string, Measurement> readBaselines(const std::string& path) {
    std::map<std::string, Measurement> out;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string name, field;
        Measurement m;
        if (!std::getline(ss, name, '|')) continue;
        auto num = [&](double& v) { if (std::getline(ss, field, '|')) v = std::atof(field.c_str()); };
        num(m.peakKW); num(m.peakKWrpm); num(m.peakNm); num(m.peakNmRpm);
        num(m.idleLo); num(m.idleHi); num(m.idleMean);
        out[name] = m;
    }
    return out;
}

bool near(double a, double b, double tol) {
    return std::abs(a - b) <= std::abs(b) * tol + 1e-9;
}

} // namespace

// Write the presets out as files. Run once against a build that still has the
// compiled-in table, after which the table is only a fallback.
int exportPresets(const char* dir) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    for (int i = 0; i < sim::presetCount(); ++i) {
        const sim::EngineDesign d = sim::preset(i);
        std::string slug;
        for (char ch : d.name) {
            if (std::isalnum(static_cast<unsigned char>(ch))) slug += static_cast<char>(std::tolower(ch));
            else if (!slug.empty() && slug.back() != '-') slug += '-';
        }
        while (!slug.empty() && slug.back() == '-') slug.pop_back();
        char num[8];
        std::snprintf(num, sizeof(num), "%02d-", i);
        const std::string path = std::string(dir) + "/" + num + slug + ".json";
        if (!sim::saveDesignJson(d, path)) {
            std::printf("could not write %s\n", path.c_str());
            return 1;
        }
        std::printf("wrote %s\n", path.c_str());
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--export-presets") == 0)
        return exportPresets(argc > 2 ? argv[2] : "presets");
    const bool update = argc > 1 && std::strcmp(argv[1], "--update") == 0;
    const bool list   = argc > 1 && std::strcmp(argv[1], "--list") == 0;
    const std::string path = baselinePath(argv[0]);
    const std::map<std::string, Measurement> base = update ? std::map<std::string, Measurement>{}
                                                           : readBaselines(path);

    std::vector<std::pair<std::string, Measurement>> results;
    int failures = 0;

    for (int i = 0; i < sim::presetCount(); ++i) {
        const sim::EngineDesign d = sim::preset(i);
        const Measurement m = measure(d);
        results.emplace_back(d.name, m);

        std::printf("%-32s %6.1f kW @ %5.0f  %7.1f Nm @ %5.0f  idle %4.0f (%4.0f-%4.0f)",
                    d.name.c_str(), m.peakKW, m.peakKWrpm, m.peakNm, m.peakNmRpm,
                    m.idleMean, m.idleLo, m.idleHi);

        if (m.stalled) {
            std::printf("   STALLED\n");
            ++failures;
            continue;
        }
        if (update || list) { std::printf("\n"); continue; }

        const auto it = base.find(d.name);
        if (it == base.end()) {
            std::printf("   new, no baseline\n");
            continue;
        }
        const Measurement& b = it->second;
        std::string bad;
        if (!near(m.peakKW, b.peakKW, kPowerTol))      bad += " power";
        if (!near(m.peakKWrpm, b.peakKWrpm, kSpeedTol)) bad += " power-rpm";
        if (!near(m.peakNm, b.peakNm, kPowerTol))      bad += " torque";
        if (!near(m.peakNmRpm, b.peakNmRpm, kSpeedTol)) bad += " torque-rpm";
        if (std::abs(m.idleMean - b.idleMean) > kIdleTol) bad += " idle";
        if (bad.empty()) {
            std::printf("   ok\n");
        } else {
            std::printf("   CHANGED:%s\n", bad.c_str());
            std::printf("%-32s was %6.1f kW @ %5.0f  %7.1f Nm @ %5.0f  idle %4.0f-%4.0f\n",
                        "", b.peakKW, b.peakKWrpm, b.peakNm, b.peakNmRpm, b.idleLo, b.idleHi);
            ++failures;
        }
    }

    if (update) {
        std::ofstream out(path);
        out << "# Recorded behaviour of every preset. Regenerate with\n"
               "#   enginio_validate --update\n"
               "# after a change you believe is an improvement, and put the diff in\n"
               "# the commit - it is the clearest statement of what the change did.\n"
               "# name|peak kW|at rpm|peak Nm|at rpm|idle low|idle high\n";
        for (const auto& r : results)
            out << r.first << '|' << std::round(r.second.peakKW * 10) / 10 << '|'
                << std::round(r.second.peakKWrpm) << '|'
                << std::round(r.second.peakNm * 10) / 10 << '|'
                << std::round(r.second.peakNmRpm) << '|'
                << std::round(r.second.idleLo) << '|'
                << std::round(r.second.idleHi) << '|'
                << std::round(r.second.idleMean) << '\n';
        std::printf("\nwrote %s\n", path.c_str());
        return 0;
    }
    if (list) return 0;

    std::printf("\n%d of %d presets changed or stalled\n", failures, sim::presetCount());
    return failures == 0 ? 0 : 1;
}
