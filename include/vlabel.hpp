#pragma once
#include "uix.hpp"
#include "gfx.hpp"


/// @brief A vector label for displaying text
/// @tparam ControlSurfaceType The UIX surface type to bind to
template<typename ControlSurfaceType>
class vlabel : public uix::canvas_control<ControlSurfaceType> {
    using base_type = uix::canvas_control<ControlSurfaceType>;
public:
    using type = vlabel;
    using control_surface_type = ControlSurfaceType;
private:
    gfx::canvas_text_info m_label_text;
    gfx::canvas_path m_label_text_path;
    gfx::rectf m_label_text_bounds;
    bool m_label_text_dirty;
    gfx::vector_pixel m_color;
    gfx::rgba_pixel<32> m_background_color;
    gfx::stream* m_font_stream;
    uix::uix_justify m_text_justify;
    gfx::matrix m_matrix;
    float m_font_size;
    gfx::sizef m_background_round;
    void build_label_path_untransformed() {
        if(m_font_stream==nullptr) {
            return;
        }
        m_label_text.ttf_font = m_font_stream;
        if(m_font_size<=0.f) {
            const float target_width = this->dimensions().width;
            float fsize = this->dimensions().height*.8f;
            m_label_text_path.initialize();
            do {
                m_label_text_path.clear();
                m_label_text.font_size = fsize;
                m_label_text_path.text({0.f,0.f},m_label_text);
                m_label_text_bounds = m_label_text_path.bounds(true);
                --fsize;
                
            } while(fsize>0.f && m_label_text_bounds.width()>=target_width);
        } else {
            m_label_text_path.initialize();
            m_label_text_path.clear();
            m_label_text.font_size = m_font_size;
            m_label_text_path.text({0.f,0.f},m_label_text);
            m_label_text_bounds = m_label_text_path.bounds(true);
            
        }
        m_matrix = gfx::matrix::create_identity();
        float w,h;
        switch(m_text_justify) {
            case uix::uix_justify::top_left:
                m_matrix.translate_inplace(-m_label_text_bounds.x1,(-m_label_text_bounds.y1));
                break;
            case uix::uix_justify::top_middle:
                m_matrix.translate_inplace(-m_label_text_bounds.x1,(-m_label_text_bounds.y1));
                w=(this->dimensions().width-m_label_text_bounds.width())*.5f;
                m_matrix.translate_inplace(w,0);
                break;
            case uix::uix_justify::top_right:
                m_matrix.translate_inplace(-m_label_text_bounds.x1,(-m_label_text_bounds.y1));
                w = this->dimensions().width - m_label_text_bounds.width();
                m_matrix.translate_inplace(w,0);
                break;
            case uix::uix_justify::center_left:
                m_matrix.translate_inplace(-m_label_text_bounds.x1,(-m_label_text_bounds.y1));
                h=(this->dimensions().height-m_label_text_bounds.height())*.5f;
                m_matrix.translate_inplace(0,h);
                break;
            case uix::uix_justify::center:
                m_matrix.translate_inplace(-m_label_text_bounds.x1,(-m_label_text_bounds.y1));
                w=(this->dimensions().width-m_label_text_bounds.width())*.5f;
                h=(this->dimensions().height-m_label_text_bounds.height())*.5f;
                m_matrix.translate_inplace(w,h);
                break;
            case uix::uix_justify::center_right:
                m_matrix.translate_inplace(0,(-m_label_text_bounds.y1));
                w = this->dimensions().width - m_label_text_bounds.width();
                h=(this->dimensions().height-m_label_text_bounds.height())*.5f;
                m_matrix.translate_inplace(w,h);
                break;
            case uix::uix_justify::bottom_left:
                m_matrix.translate_inplace(-m_label_text_bounds.x1,(-m_label_text_bounds.y1));
                h = this->dimensions().height - m_label_text_bounds.height();
                m_matrix.translate_inplace(0,h);
                break;
            case uix::uix_justify::bottom_middle:
                m_matrix.translate_inplace(-m_label_text_bounds.x1,(-m_label_text_bounds.y1));
                w=(this->dimensions().width-m_label_text_bounds.width())*.5f;
                h = this->dimensions().height - m_label_text_bounds.height();
                m_matrix.translate_inplace(w,h);
                break;
            default: //uix::uix_justify::bottom_right:
                w = this->dimensions().width - m_label_text_bounds.width();
                h = this->dimensions().height - m_label_text_bounds.height();
                m_matrix.translate_inplace(w,h);
                break;
        }
        
    }
public:
    vlabel() : base_type() ,m_label_text_dirty(true) {
        m_font_stream = nullptr;
        m_label_text.text_sz("Label");
        m_label_text.encoding = &gfx::text_encoding::utf8;
        m_label_text.ttf_font_face = 0;
        m_color = gfx::vector_pixel(255,255,255,255);
        m_background_color = gfx::rgba_pixel<32>(0,true);
        m_font_size = 0.f;
        m_background_round = {0.f,0.f};
    }
    virtual ~vlabel() {

    }
    uix::uix_justify text_justify() const {
        return m_text_justify;
    }
    void text_justify(uix::uix_justify value) {
        if(m_text_justify!=value) {
            m_text_justify = value;
            m_label_text_dirty = true;
            this->invalidate();
        }
    }
    gfx::text_handle text() const {
        return m_label_text.text;
    }
    void text(gfx::text_handle text, size_t text_byte_count) {
        m_label_text.text=text;
        m_label_text.text_byte_count = text_byte_count;
        m_label_text_dirty = true;
        this->invalidate();
    }
    void text(const char* sz) {
        m_label_text.text_sz(sz);
        m_label_text_dirty = true;
        this->invalidate();
    }
        gfx::stream& font() const {
        return *m_font_stream;
    }
    void font(gfx::stream& value) {
        m_font_stream = &value;
        m_label_text_dirty = true;
        this->invalidate();
    }
    float font_size() const {
        return m_font_size;
    }
    void font_size(float value) {
        m_font_size = value;
        m_label_text_dirty = true;
        this->invalidate();
    }
    gfx::rgba_pixel<32> color() const {
        gfx::rgba_pixel<32> result;
        convert(m_color,&result);
        return result;
    }
    void color(gfx::rgba_pixel<32> value) {
        convert(value,&m_color);
        this->invalidate();
    }
    gfx::rgba_pixel<32> background_color() const {
        return m_background_color;
    }
    void background_color(gfx::rgba_pixel<32> value) {
        m_background_color = value;
        this->invalidate();
    }
    gfx::sizef background_round() const {
        return m_background_round;
    }
    void background_round(gfx::sizef value) {
        m_background_round=value;
        this->invalidate();
    }
    
protected:
    virtual void on_before_paint() override {
        if(m_label_text_dirty) {
            build_label_path_untransformed();
            m_label_text_dirty = false;
        }
    }
    virtual void on_paint(control_surface_type& destination, const gfx::srect16& clip) {
        if(m_background_color.opacity()!=0) {
            if(m_background_round.width==0.f && m_background_round.height==0.f ) {
                gfx::draw::filled_rectangle(destination,destination.bounds(),m_background_color);
            }
        }
        base_type::on_paint(destination,clip);
    }
    virtual void on_paint(gfx::canvas& destination, const gfx::srect16& clip) override {
        if(m_font_stream==nullptr) {
            return;
        }
        // save the current transform
        gfx::matrix old = destination.transform();
        destination.transform(gfx::matrix::create_identity());
        gfx::canvas_style si = destination.style();
        si.fill_paint_type = gfx::paint_type::solid;
        si.stroke_paint_type = gfx::paint_type::none;
        if(m_background_color.opacity()!=0) {
            if(m_background_round.width!=0.f || m_background_round.height!=0.f ) {
                gfx::vector_pixel bg;
                convert(m_background_color,&bg);
                si.fill_color = bg;
                destination.style(si);
                destination.rounded_rectangle((gfx::rectf)destination.bounds(),m_background_round);
                destination.render();
            }
        }
        si.fill_color = m_color;
        destination.style(si);
        destination.transform(m_matrix);
        destination.path(m_label_text_path);
        destination.render();
        // restore the old transform
        destination.transform(old);
    }
};

