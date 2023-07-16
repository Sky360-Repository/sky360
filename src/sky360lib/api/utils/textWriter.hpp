#pragma once

#include <opencv2/highgui.hpp>
#include <string>

namespace sky360lib::utils
{
    class TextWriter
    {
    public:
        TextWriter(cv::Scalar _color = cv::Scalar{80, 140, 190, 0}, int _num_max_lines = 32, double _thickness_scale = 3.5)
        : m_font_face{cv::FONT_HERSHEY_SIMPLEX}
        , m_num_lines{_num_max_lines}
        , m_color{_color}
        , m_color16{_color[0] * 255, _color[1] * 255, _color[2] * 255, _color[3] * 255}
        , m_thickness_scale{_thickness_scale}
        {
            m_max_height = get_max_text_height();
            m_horizontal_padding = m_max_height / 2;
        }

        void write_text(const cv::Mat _frame, std::string _text, int _line, bool _align_right = false) const
        {
            const double font_scale = calc_font_scale(m_max_height, _frame.size().height);
            const int thickness = (int)(m_thickness_scale * font_scale);
            const int height = calc_height(_line, _frame.size().height);
            int posX = !_align_right ? m_horizontal_padding : _frame.size().width - (get_text_size(_text, font_scale, thickness).width + m_horizontal_padding);

            cv::putText(_frame, _text, cv::Point(posX, height), m_font_face, font_scale, _frame.elemSize1() == 1 ? m_color : m_color16, thickness, cv::LINE_AA);
        }

    private:
        const int m_font_face;
        const int m_num_lines;
        const cv::Scalar m_color;
        const cv::Scalar m_color16;
        double m_thickness_scale;
        int m_max_height;
        int m_horizontal_padding;

        inline cv::Size get_text_size(const std::string& _text, double _font_scale, int _thickness) const
        {
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(_text, m_font_face, _font_scale, _thickness, &baseline);
            text_size.height += baseline;
            return text_size;
        }

        inline int get_max_text_height() const
        {
            const std::string text = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz(){}[]!|$#^0123456789";
            return get_text_size(text, 1.0, 5).height;
        }

        inline double calc_font_scale(int _font_height, uint32_t _screen_height) const
        {
            const double line_height = (double)_screen_height / (double)m_num_lines;
            return line_height / (double)_font_height;
        }

        inline int calc_height(int _line, uint32_t _screen_height) const
        {
            const int line_height = _screen_height / m_num_lines;
            return _line * line_height;
        }
    };
}