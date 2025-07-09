#pragma once
#include <gtkmm.h>
#include <vector>

class SharpnessGraph : public Gtk::DrawingArea {
public:
    SharpnessGraph();
    virtual ~SharpnessGraph();
    
    // Update the graph with new sharpness values
    void updateValues(const std::vector<double>& values);
    
    // Set the maximum value for scaling
    void setMaxValue(double max);

protected:
    // Override default signal handler
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
    
private:
    std::vector<double> m_values;
    double m_maxValue;
    Glib::Dispatcher m_signalDataChanged;
    
    void onDataChanged();
}; 