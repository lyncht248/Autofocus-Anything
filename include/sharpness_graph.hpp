#pragma once
#include <gtkmm.h>
#include <vector>

/**
 * @class SharpnessGraph
 * @brief A custom GTK drawing area for visualizing sharpness values.
 * 
 * The SharpnessGraph class provides functionality to display a graph of sharpness values,
 * including grid lines, scaling, and dynamic updates.
 */
class SharpnessGraph : public Gtk::DrawingArea {
public:
    /**
     * @brief Constructor for the SharpnessGraph class.
     * 
     * Initializes the drawing area with default settings and connects the data changed signal.
     */
    SharpnessGraph();

    /**
     * @brief Destructor for the SharpnessGraph class.
     */
    virtual ~SharpnessGraph();

    /**
     * @brief Updates the graph with new sharpness values.
     * 
     * Thread-safe method to update the sharpness values and schedule a redraw.
     * @param values Vector of sharpness values to display.
     */
    void updateValues(const std::vector<double>& values);

    /**
     * @brief Sets the maximum value for scaling the graph.
     * 
     * Ensures the maximum value is positive and updates the scaling factor.
     * @param max Maximum value for scaling.
     */
    void setMaxValue(double max);

protected:
    /**
     * @brief Overrides the default signal handler for drawing.
     * 
     * Handles the drawing of the graph, including background, grid lines, and sharpness curve.
     * @param cr Cairo context used for drawing.
     * @return True if the drawing is successful.
     */
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;

private:
    std::vector<double> m_values; ///< Sharpness values to be displayed on the graph.
    double m_maxValue; ///< Maximum value for scaling the graph.
    Glib::Dispatcher m_signalDataChanged; ///< Signal dispatcher for scheduling redraws.

    /**
     * @brief Handles the data changed signal.
     * 
     * Schedules a redraw of the widget when the sharpness values are updated.
     */
    void onDataChanged();
};