#ifndef SRC_VISUALIZER_GRAPHWIDGET_H_
#define SRC_VISUALIZER_GRAPHWIDGET_H_

#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtGui/QWheelEvent>
#include <QtWidgets/QMenu>
#include <QtWidgets/QWidget>
#include <memory>
#include <cmath>

/** \brief plotting of timeseries data.
 *
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/
class GraphWidget : public QWidget {
  Q_OBJECT
 public:
  /** \brief locations, scaling, etc. **/
  class GraphContext {
   public:
    QPainter* painter;

    int32_t t_current;    // currently right, most timestamp.
    float width, height;  // width and height of the widget.
    float deltaT;         // pixels between two timesteps or values.

    float margin;

    bool isNearestToCursor;
    QPoint cursor;
    bool showLegend;
  };

  class Timeseries {
   public:
    typedef std::shared_ptr<Timeseries> Ptr;

    Timeseries(const QColor& color, const Qt::PenStyle& style = Qt::SolidLine);

    void clear();

    void draw(const GraphContext& ctx);

    void insert(uint32_t t, double value);

    double minimum();
    double maximum();

    /** \brief set minimum visible value **/
    void setMinimum(double value);
    /** \brief set maximum visible value **/
    void setMaximum(double value);

    /** \brief set the graph minimum/maximum fixed.
     *   A fixed timeseries will not adapt it's minimum/maximum value to include all values.
     */
    void setFixed(bool value);
    bool isFixed() const;

    /** \brief add a threshold at given value. **/
    void addThreshold(double value);

    /** \brief remove threshold for given value.
     *
     *  Removes all thresholds at a specific value and its epsilon surrounding.
     *
     *  \param value for threshold(s) to remove
     *  \param epsilon for remove of threshold
     *
     **/
    void removeThreshold(double value, double epsilon = 0.0001);

    /** \brief drop all thresholds. **/
    void removeAllThresholds();

    /** \brief distance of given point to times series. **/
    float distance(const GraphContext& ctx, QPoint& cursor);

    const QColor& color() const;
    const Qt::PenStyle& style() const;

   protected:
    QColor color_;
    Qt::PenStyle style_;
    std::vector<double> values_;
    std::vector<double> thresholds_;

    double min_{0}, max_{0};
    bool isFixed_{false};
  };

  class Marker {
   public:
    Marker(uint32_t t, const QColor& color);
    uint32_t t;
    QColor color;
  };

  GraphWidget(QWidget* parent = 0, Qt::WindowFlags f = 0);

  /** \brief reset current view and clear all timeseries. **/
  void reset();

  /** \brief add a timeseries for drawing
   *
   *  \return just created timeseries.
   **/
  Timeseries::Ptr addTimeseries(const std::string& name, const QColor& color,
                                const Qt::PenStyle& style = Qt::SolidLine);

  /** \brief get a timeseries for given name and fails if timeseries not available.
   *  \throws a std::runtime_error
   **/
  Timeseries::Ptr getTimeseries(const std::string& name);

  std::map<std::string, Timeseries::Ptr>& getAllTimeseries();

  /** \brief set current visible timestamp to this position. **/
  void setTimestamp(uint32_t t);

  /** \brief add a marker at a specific time. **/
  void insertMarker(uint32_t t, const QColor& color);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void contextMenuEvent(QContextMenuEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

  void handleContextAction(const std::string& target, bool checked);

  std::vector<std::string> order_;
  std::map<std::string, Timeseries::Ptr> timeseries_;
  std::map<std::string, bool> visible_;

  // currently visible area.
  uint32_t t_current_{0};
  int32_t margin_;

  float visibleTimestamps_{20.0f};

  std::vector<Marker> markers_;

  QPoint currentCursor_;
  bool hasFocus_{false};
  bool fixedPosition_{false};
  int32_t nearestIndex_{-1};

  bool showLegend_{true};

  QMenu contextMenu_;
};

#endif /* SRC_VISUALIZER_GRAPHWIDGET_H_ */
