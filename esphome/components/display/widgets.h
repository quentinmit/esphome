#pragma once

#include <bits/stdc++.h>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#include "esphome/components/display/display_buffer.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace display {

class Widget {
 public:
  void set_preferred_size(int width, int height) {
    user_preferred_width_ = width;
    user_preferred_height_ = height;
  };
  void set_maximum_size(int width, int height) {
    user_maximum_width_ = width;
    user_maximum_height_ = height;
  };
  void get_minimum_size(int *width, int *height) {
    *width = minimum_width_;
    *height = minimum_height_;
  };
  void get_preferred_size(int *width, int *height) {
    *width = (user_preferred_width_ >= 0) ? user_preferred_width_ : preferred_width_;
    *height = (user_preferred_height_ >= 0) ? user_preferred_height_ : preferred_height_;
  };
  void get_maximum_size(int *width, int *height) {
    *width = (user_maximum_width_ >= 0) ? user_maximum_width_ : maximum_width_;
    *height = (user_maximum_height_ >= 0) ? user_maximum_height_ : maximum_height_;
  };

  // invalidate_layout should recalculate {minimum,preferred,maximum}_{width,height}_ as necessary.
  virtual void invalidate_layout();

  virtual void draw(DisplayBuffer* it, int x1, int y1, int width, int height) = 0;

  void draw_fullscreen(DisplayBuffer& it);
protected:
  int minimum_width_ = 0, minimum_height_ = 0;
  int preferred_width_ = 0, preferred_height_ = 0;
  int maximum_width_ = SHRT_MAX, maximum_height_ = SHRT_MAX;
private:
  int user_preferred_width_ = -1, user_preferred_height_ = -1;
  int user_maximum_width_ = -1, user_maximum_height_ = -1;
protected:
  struct SizeRequirements {
  public:
    int minimum = 0;
    int preferred = 0;
    int maximum = 0;
    float alignment = 0.5;
    static SizeRequirements get_tiled_size_requirements(std::vector<SizeRequirements> children);
    static SizeRequirements get_aligned_size_requirements(std::vector<SizeRequirements> children);
    void calculate_tiled_positions(int allocated, std::vector<SizeRequirements> children, std::vector<int> &offsets, std::vector<int> &spans);
    void calculate_aligned_positions(int allocated, std::vector<SizeRequirements> children, std::vector<int> &offsets, std::vector<int> &spans);
  };
};

class WidgetContainer : public Widget {
 public:
  virtual void invalidate_layout();
  virtual void draw(DisplayBuffer* it, int x1, int y1, int width, int height);

  void set_children(std::vector<Widget *> children) {
    children_.clear();
    std::for_each(begin(children), end(children), [this](Widget* widget) {
      children_.emplace_back(Child(widget));
    });
  };

 protected:
  class Child {
  public:
    Child(Widget* widget) : widget_(widget) {}
    Widget *widget_;
    int x, y, width, height;
  };
  std::vector<Child> children_;
};

  class Box : public WidgetContainer {
  protected:
    enum Axis {
      X_AXIS,
      Y_AXIS,
    };
    Box(Axis axis)
      : axis_(axis) {}
  public:
    virtual void invalidate_layout();
    virtual void draw(DisplayBuffer* it, int x1, int y1, int width, int height);
  private:
    std::vector<SizeRequirements> xChildren_, yChildren_;
    SizeRequirements xTotal_, yTotal_;
    Axis axis_;
  };

class Horizontal : public Box {
public:
  Horizontal() : Box(X_AXIS) {};
};

class Vertical : public Box {
public:
  Vertical() : Box(Y_AXIS) {};
};

template<typename... Ts>  class Text : public Widget {
 public:
   TEMPLATABLE_VALUE(std::string, text);
  void set_font(Font* font) { font_ = font; }
  void set_textalign(TextAlign align) { align_ = align; }

  virtual void invalidate_layout();
  virtual void draw(DisplayBuffer* it, int x1, int y1, int width, int height);

  void set_sensor(sensor::Sensor *sensor) { source_ = sensor; source_text_ = NULL; }
  void set_sensor(text_sensor::TextSensor *sensor) { source_text_ = sensor; source_ = NULL; }

 protected:
  Font* font_;
  TextAlign align_ = TextAlign::TOP_LEFT;
  void calculate_text_();
  std::string cached_text_;
  sensor::Sensor *source_;
  text_sensor::TextSensor *source_text_;
};

  class Button : public Widget {
  public:
    void set_child(Widget* child) { child_ = child; };

    virtual void invalidate_layout();
    virtual void draw(DisplayBuffer* it, int x1, int y1, int width, int height);
  protected:
    Widget* child_;
  };

}  // namespace display
}  // namespace esphome