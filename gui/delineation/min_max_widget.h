#ifndef GUI_DELINEATION_MIN_MAX_WIDGET_H
#define GUI_DELINEATION_MIN_MAX_WIDGET_H

#include <QWidget>

namespace Ui
{
  class MinMaxWidget;
}

class MinMaxWidget : public QWidget
{

    Q_OBJECT

  public:

    explicit MinMaxWidget (QWidget* parent = 0);

    ~MinMaxWidget ();

    void
    setRange (double min, double max);

    void
    setValue (double min, double max);

    void
    setSingleStep (double step);

    std::pair<double, double>
    value () const;

    std::pair<double, double>
    range () const;

    double
    singleStep () const;

  public Q_SLOTS:

    void
    onSliderMinValueChanged (int value);

    void
    onSpinboxMinValueChanged (double value);

    void
    onSliderMaxValueChanged (int value);

    void
    onSpinboxMaxValueChanged (double value);

  Q_SIGNALS:

    void
    valueChanged (double min, double max);

  private:

    /** Update values and ranges in sliders and spinboxes to reflect the
      * internal state. */
    void
    updateWidgets ();

    /** Map integer slider value to the real value range. */
    double
    toRealValue (int value);

    int
    toSliderValue (double value);

    Ui::MinMaxWidget* ui_;

    double range_min_ = 0;
    double range_max_ = 99;
    double min_ = 0;
    double max_ = 99;

    double ratio_;
    unsigned int slider_range_;
    bool updating_widgets_;

};

#endif // GUI_DELINEATION_MIN_MAX_WIDGET_H
