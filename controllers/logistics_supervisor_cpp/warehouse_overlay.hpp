#ifndef WAREHOUSE_OVERLAY_HPP
#define WAREHOUSE_OVERLAY_HPP

#include <webots/Display.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

// Presentation only: the supervisor supplies a snapshot of its existing rule
// state. The display never changes scoring, timing, or the student protocol.
namespace warehouse_overlay
{

  // Keep these dimensions in sync with the warehouse_status Display in the world.
  constexpr int kWidth = 400;
  constexpr int kHeight = 208;
  constexpr int kStatusHeight = 164;
  constexpr double kWarningDuration = 8.0;
  constexpr int kBackground = 0x15202B;
  constexpr int kPanel = 0x223140;
  constexpr int kText = 0xF4F7FA;
  constexpr int kSecondary = 0xB9C9D8;
  constexpr int kReady = 0x82E0B0;
  constexpr int kWaiting = 0xFFD180;
  constexpr int kError = 0xFF9C9C;

  struct Box
  {
    char type = 'B';
    bool delivered = false;
  };

  struct Bay
  {
    int inputBox = -1;
    int outputBox = -1;
    bool processing = false;
    double remainingSeconds = 0.0;
  };

  struct Snapshot
  {
    double time = 0.0;
    int score = 0;
    std::string initialOrder;
    bool randomOrder = true;
    bool magnetOn = false;
    std::string attachedBox = "none";
    std::array<Box, 4> boxes;
    // A0, A1, B0, B1. Bay numbers match the student API.
    std::array<Bay, 4> bays;
    std::string event;
    double eventTime = 0.0;
    bool eventWarning = false;
  };

  inline std::string clockText(double time)
  {
    const int seconds = std::max(0, static_cast<int>(time));
    char text[32];
    std::snprintf(text, sizeof(text), "%02d:%02d", seconds / 60, seconds % 60);
    return text;
  }

  inline int maximumScore(const std::string &order)
  {
    int score = 0;
    for (char type : order)
      score += type == 'R' ? 3 : type == 'G' ? 2
                                             : 1;
    return score;
  }

  class Renderer
  {
  public:
    explicit Renderer(webots::Display *display) : display_(display) {}

    void draw(const Snapshot &state)
    {
      if (!display_)
        return;

      // Clear the previous frame, including an expired warning. The unused footer
      // stays transparent so the scene is visible below the compact status list.
      display_->setOpacity(1.0);
      display_->setAlpha(0.0);
      rectangle(0, 0, kWidth, kHeight, kBackground);
      display_->setAlpha(1.0);
      const double eventAge = state.time - state.eventTime;
      const bool showWarning = state.eventWarning && !state.event.empty() &&
                               eventAge >= 0.0 && eventAge < kWarningDuration;
      rectangle(0, 0, kWidth, kHeight, kBackground);

      const int delivered = static_cast<int>(std::count_if(
          state.boxes.begin(), state.boxes.end(), [](const Box &box)
          { return box.delivered; }));
      text("Delivered " + std::to_string(delivered) + "/4", 12, 12, 16,
           delivered == 4 ? kReady : kText);
      text("Score " + std::to_string(state.score) + "/" + std::to_string(maximumScore(state.initialOrder)),
           156, 12, 16);
      // Monospaced, with space for elapsed times beyond 99 minutes.
      text(clockText(state.time), 310, 13, 14, kSecondary, "Lucida Console");

      text("Order " + state.initialOrder, 12, 39, 14);
      text(std::string("Magnet ") + (state.magnetOn ? "ON" : "OFF"), 134, 39, 14,
           state.magnetOn ? kText : kSecondary);
      text("Carrying: " + state.attachedBox, 252, 39, 14,
           state.attachedBox == "none" ? kSecondary : kText);
      rectangle(12, 63, kWidth - 24, 1, kPanel);

      for (int machine = 0; machine < 2; ++machine)
        for (int bay = 0; bay < 2; ++bay)
          drawBay(state.bays[2 * machine + bay], machine, bay, 75 + 21 * (2 * machine + bay));

      if (showWarning)
        drawWarning(state.event);
    }

  private:
    void drawWarning(const std::string &event)
    {
      rectangle(12, kStatusHeight, kWidth - 24, 1, kPanel);
      // Only rejected actions need an event notice; routine events are already
      // represented by the status rows. Wrap at words and cap the two-line footer.
      // The bundled fixed-width font keeps the fit predictable.
      std::string message = event;
      constexpr size_t kLineLength = 46;
      for (int line = 0; line < 2 && !message.empty(); ++line)
      {
        if (line == 1 && message.size() > kLineLength)
        {
          text(message.substr(0, kLineLength - 3) + "...", 12, 174 + 17 * line, 12, kError, "Lucida Console");
          break;
        }
        size_t count = message.size();
        if (count > kLineLength)
        {
          count = message.rfind(' ', kLineLength);
          if (count == std::string::npos)
            count = kLineLength;
        }
        text(message.substr(0, count), 12, 174 + 17 * line, 12, kError, "Lucida Console");
        message.erase(0, count);
        if (!message.empty() && message.front() == ' ')
          message.erase(0, 1);
      }
    }

    void text(const std::string &value, int x, int y, int size, int color = kText,
              const std::string &font = "Arial")
    {
      display_->setColor(color);
      display_->setFont(font, size, true);
      display_->drawText(value, x, y);
    }

    void rectangle(int x, int y, int width, int height, int color)
    {
      display_->setColor(color);
      display_->fillRectangle(x, y, width, height);
    }

    void drawBay(const Bay &bay, int machine, int index, int y)
    {
      text(std::string(machine == 0 ? "A" : "B") + " / Bay " + std::to_string(index), 12, y, 14, kSecondary);

      std::string status = "Idle";
      int color = kSecondary;
      if (bay.processing && bay.inputBox >= 0)
      {
        status = "BOX_" + std::to_string(bay.inputBox) + " processing " +
                 std::to_string(static_cast<int>(std::ceil(std::max(0.0, bay.remainingSeconds)))) + " s";
        color = kWaiting;
      }
      else if (bay.inputBox >= 0 && bay.outputBox >= 0)
      {
        status = "BOX_" + std::to_string(bay.inputBox) + " waiting / BOX_" + std::to_string(bay.outputBox) + " ready";
        color = kWaiting;
      }
      else if (bay.outputBox >= 0)
      {
        status = "BOX_" + std::to_string(bay.outputBox) + " ready";
        color = kReady;
      }
      else if (bay.inputBox >= 0)
      {
        status = "BOX_" + std::to_string(bay.inputBox) + " waiting";
        color = kWaiting;
      }
      text(status, 92, y, 14, color);
    }

    webots::Display *display_;
  };

} // namespace warehouse_overlay

#endif
