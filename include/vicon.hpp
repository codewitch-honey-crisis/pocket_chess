#pragma once
#include "gfx.hpp"
#include "uix.hpp"

/// @brief An SVG icon
/// @tparam ControlSurfaceType The UIX surface type to bind to
template <typename ControlSurfaceType>
class vicon : public uix::control<ControlSurfaceType> {
    using base_type = uix::control<ControlSurfaceType>;

   public:
    typedef void (*on_pressed_changed_callback_type)(bool pressed, void* state);

   private:
    bool m_pressed;
    bool m_dirty;
    gfx::sizef m_svg_dim;
    gfx::matrix m_fit;
    on_pressed_changed_callback_type m_on_pressed_changed_callback;
    void* m_on_pressed_changed_callback_state;
    gfx::stream* m_svg;
    gfx::rgba_pixel<32> m_background_color;
    
    static gfx::rectf correct_aspect(gfx::srect16& sr, float aspect) {
        if (aspect>=1.f) {
            sr.y2 /= aspect;
        } else {
            sr.x2 *= aspect;
        }
        return (gfx::rectf)sr;
    }

   public:
    vicon()
        : base_type(),
          m_pressed(false),
          m_dirty(true),
          m_on_pressed_changed_callback(nullptr),
          m_svg(nullptr) {m_svg_dim = {0.f,0.f};}
    gfx::stream& svg() const { return *m_svg; }
    void svg(gfx::stream& value) {
        m_svg = &value;
        m_dirty = true;
        this->invalidate();
    }
    gfx::sizef svg_dimensions() const { return m_svg_dim; }
    void svg_dimensions(const gfx::sizef& value) {
        m_svg_dim = value;
        m_dirty = true;
        this->invalidate();
    }
    gfx::rgba_pixel<32> background_color() const {
        return m_background_color;
    }
    void background_color(gfx::rgba_pixel<32> value) {
        m_background_color = value;
        this->invalidate();
    }
    bool pressed() const { return m_pressed; }
    void on_pressed_changed_callback(on_pressed_changed_callback_type callback,
                                     void* state = nullptr) {
        m_on_pressed_changed_callback = callback;
        m_on_pressed_changed_callback_state = state;
    }

   protected:
    virtual void on_before_paint() override {
        if (m_dirty) {
            if (m_svg != nullptr) {
                if(m_svg_dim.width==0.f || m_svg_dim.height==0.f) {
                    m_svg->seek(0);
                    gfx::canvas::svg_dimensions(*m_svg, &m_svg_dim);
                }
                gfx::srect16 sr = this->dimensions().bounds();
                sr.inflate_inplace(-1,-1);
                gfx::rectf corrected = correct_aspect(sr, m_svg_dim.aspect_ratio());
                corrected.center_inplace((gfx::rectf)sr);
                m_fit = gfx::matrix::create_fit_to(m_svg_dim,corrected);
            }
            m_dirty = false;
        }
    }
    virtual void on_paint(ControlSurfaceType& dst,
                          const gfx::srect16& clip) override {
        if (m_dirty || m_svg == nullptr) {
           
            return;
        }
        if(m_background_color.opacity()!=0) {
            gfx::draw::filled_rectangle(dst,dst.bounds(),m_background_color);
        }
        gfx::canvas cvs((gfx::size16)this->dimensions());
        cvs.initialize();
        gfx::spoint16 pt(m_pressed,m_pressed);
        gfx::draw::canvas(dst, cvs,pt);
        m_svg->seek(0);
        if (gfx::gfx_result::success != cvs.render_svg(*m_svg, m_fit)) {
            puts("SVG render error");
        }
    }
    virtual bool on_touch(size_t locations_size,
                          const gfx::spoint16* locations) override {
        if (!m_pressed) {
            m_pressed = true;
            if (m_on_pressed_changed_callback != nullptr) {
                m_on_pressed_changed_callback(
                    true, m_on_pressed_changed_callback_state);
            }
            this->invalidate();
        }
        return true;
    }
    virtual void on_release() override {
        if (m_pressed) {
            m_pressed = false;
            if (m_on_pressed_changed_callback != nullptr) {
                m_on_pressed_changed_callback(
                    false, m_on_pressed_changed_callback_state);
            }
            this->invalidate();
        }
    }
};
