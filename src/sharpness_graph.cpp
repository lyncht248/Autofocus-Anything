#include "sharpness_graph.hpp"

SharpnessGraph::SharpnessGraph() : 
    m_maxValue(1.0)
{
    set_size_request(250, 55); // Reduced height for 2-row span
    set_vexpand(false); // Prevent vertical expansion
    set_hexpand(false); // Also prevent horizontal expansion
    set_valign(Gtk::Align::ALIGN_START); // Align to top instead of filling
    set_halign(Gtk::Align::ALIGN_START); // Align to left instead of filling
    
    // Connect the dispatcher to the redraw function
    m_signalDataChanged.connect(sigc::mem_fun(*this, &SharpnessGraph::onDataChanged));
}

SharpnessGraph::~SharpnessGraph() {
}

void SharpnessGraph::updateValues(const std::vector<double>& values) {
    // Thread-safe way to update the values
    m_values = values;
    m_signalDataChanged.emit();
}

void SharpnessGraph::setMaxValue(double max) {
    m_maxValue = max > 0 ? max : 1.0;
}

void SharpnessGraph::onDataChanged() {
    // Schedule a redraw of the widget
    queue_draw();
}

bool SharpnessGraph::on_draw(const Cairo::RefPtr<Cairo::Context>& cr) {
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    
    // Fill background with GUI-matching grey (typical GTK gray)
    cr->set_source_rgb(0.2, 0.2, 0.2);  // Light grey to match GUI background
    cr->rectangle(0, 0, width, height);
    cr->fill();
    
    // Draw grid lines with slightly darker grey
    cr->set_source_rgba(0.7, 0.7, 0.7, 0.6);
    cr->set_line_width(0.5);
    
    // Horizontal grid lines
    for (int i = 1; i < 4; i++) {
        double y = height * i / 4.0;
        cr->move_to(0, y);
        cr->line_to(width, y);
    }
    
    // Vertical grid lines
    for (int i = 1; i < 10; i++) {
        double x = width * i / 10.0;
        cr->move_to(x, 0);
        cr->line_to(x, height);
    }
    cr->stroke();
    
    // Draw sharpness curve with white line instead of green
    if (m_values.size() > 1) {
        cr->set_source_rgb(1.0, 1.0, 1.0);  // White
        cr->set_line_width(1.5);
        
        const double step = width / static_cast<double>(m_values.size() - 1);
        
        cr->move_to(0, height - (m_values[0] / m_maxValue) * height);
        
        for (size_t i = 1; i < m_values.size(); i++) {
            double x = i * step;
            double y = height - (m_values[i] / m_maxValue) * height;
            cr->line_to(x, y);
        }
        cr->stroke();
    }
    
    // Draw horizontal line if values are close to even (orthogonal to subject)
    if (m_values.size() > 2) {
        bool isEven = true;
        double avg = 0;
        for (auto val : m_values) {
            avg += val;
        }
        avg /= m_values.size();
        
        // Check if values are within 10% of average
        for (auto val : m_values) {
            if (std::abs(val - avg) > 0.1 * avg) {
                isEven = false;
                break;
            }
        }
        
        if (isEven) {
            // yellow color for the horizontal even line
            //cr->set_source_rgb(1.0, 1.0, 0.0);  // Yellow
            cr->set_source_rgb(0.0, 0.8, 1.0);  // Cyan color for the horizontal even line
            cr->set_line_width(2.0);
            double y = height - (avg / m_maxValue) * height;
            cr->move_to(0, y);
            cr->line_to(width, y);
            cr->stroke();
        }
    }
    
    return true;
} 