#include "GraphWidget.h"

#include <iostream>

GraphWidget::Timeseries::Timeseries(const QColor& color, const Qt::PenStyle& style) : color_(color), style_{style} {}

void GraphWidget::Timeseries::draw(const GraphContext& ctx) {
  QPainter& painter = *ctx.painter;

  QPen pen(color_);
  pen.setStyle(style_);
  painter.setPen(pen);

  QPoint last;
  bool firstTime = true;
  float deltaY = (ctx.height - 2 * ctx.margin) / (max_ - min_);
  float baseline = ctx.height - ctx.margin;

  for (int32_t t = values_.size() - 1; t >= 0; --t) {
    int32_t idx = ctx.t_current - t;
    int32_t x = ctx.width - idx * ctx.deltaT - ctx.margin;
    int32_t y = baseline - deltaY * (values_[t] - min_);

    if (!firstTime) painter.drawLine(x, y, last.x(), last.y());

    last = QPoint(x, y);

    firstTime = false;

    if (x < 0) break;  // just go back as long as it
  }

  // draw the thresholds.
  for (uint32_t i = 0; i < thresholds_.size(); ++i) {
    int32_t y = baseline - deltaY * (thresholds_[i] - min_);
    painter.drawLine(0, y, ctx.width, y);
  }

  pen.setStyle(Qt::SolidLine);
  painter.setPen(pen);

  if (ctx.isNearestToCursor) {
    float t = ctx.t_current - float(ctx.width - ctx.cursor.x() - ctx.margin) / ctx.deltaT;

    int32_t t_1 = std::max<int32_t>(std::floor(t), 0);
    int32_t t_2 = std::min<int32_t>(std::ceil(t), values_.size() - 1);

    float x_1 = ctx.width - (ctx.t_current - t_1) * ctx.deltaT - ctx.margin;
    float x_2 = ctx.width - (ctx.t_current - t_2) * ctx.deltaT - ctx.margin;
    float y_1 = baseline - deltaY * (values_[t_1] - min_);
    float y_2 = baseline - deltaY * (values_[t_2] - min_);

    float lambda = (t_2 > t_1) ? ((ctx.cursor.x() - x_1) / (x_2 - x_1)) : 0;
    lambda = std::max(0.0f, std::min(1.0f, lambda));  // clamp to valid range.

    float y_t = y_1 + lambda * (y_2 - y_1);

    float value = (baseline - y_t) / deltaY + min_;

    float x = ctx.cursor.x();
    float y = y_t;

    painter.setBrush(color_);
    painter.drawEllipse(x - 2, y - 2, 4, 4);

    QString txt = QString::number((double)value, 'g', 2);
    QRectF bbox = painter.boundingRect(x + 2, y + 4, 2, 2, Qt::AlignLeft, txt);

    double x_txt = x + 2;
    double y_txt = y - 2;
    if (x_txt + bbox.width() + ctx.margin > ctx.width) x_txt = x - bbox.width() - 2;
    if (y_txt < ctx.margin) y_txt = y + bbox.height();

    painter.drawText(x_txt, y_txt, txt);
  }
}

void GraphWidget::Timeseries::clear() {
  values_.clear();
}

void GraphWidget::Timeseries::insert(uint32_t t, double value) {
  if (!isFixed_) {
    min_ = std::min(value, min_);
    max_ = std::max(value, max_);
  }
  // ensure that missing timestamps get filled in.
  double init = (values_.size() > 0) ? values_.back() : 0.0;
  values_.resize(std::max<int32_t>(t + 1, values_.size()), init);
  values_[t] = value;
}

double GraphWidget::Timeseries::minimum() {
  return min_;
}

double GraphWidget::Timeseries::maximum() {
  return max_;
}

void GraphWidget::Timeseries::setMinimum(double value) {
  min_ = value;
}

void GraphWidget::Timeseries::setMaximum(double value) {
  max_ = value;
}

void GraphWidget::Timeseries::addThreshold(double value) {
  thresholds_.push_back(value);
  if (!isFixed_) {
    min_ = std::min(value, min_);
    max_ = std::max(value, max_);
  }
}

void GraphWidget::Timeseries::removeThreshold(double value, double epsilon) {
  std::vector<double> temp;

  for (auto t : thresholds_)
    if (std::abs(t - value) > epsilon) temp.push_back(t);

  thresholds_ = temp;
}

void GraphWidget::Timeseries::removeAllThresholds() {
  thresholds_.clear();
}

float GraphWidget::Timeseries::distance(const GraphContext& ctx, QPoint& cursor) {
  if (values_.size() == 0) return 1000;

  float deltaY = (ctx.height - 2 * ctx.margin) / (max_ - min_);
  float baseline = ctx.height - ctx.margin;

  float t = ctx.t_current - (ctx.width - cursor.x() - ctx.margin) / ctx.deltaT;

  int32_t t_1 = std::max<int32_t>(std::floor(t), 0);
  int32_t t_2 = std::min<int32_t>(std::ceil(t), values_.size() - 1);

  float x_1 = ctx.width - (ctx.t_current - t_1) * ctx.deltaT - ctx.margin;
  float x_2 = ctx.width - (ctx.t_current - t_2) * ctx.deltaT - ctx.margin;
  float y_1 = baseline - deltaY * (values_[t_1] - min_);
  float y_2 = baseline - deltaY * (values_[t_2] - min_);

  float lambda = (t_2 > t_1) ? ((ctx.cursor.x() - x_1) / (x_2 - x_1)) : 0;
  lambda = std::max(0.0f, std::min(1.0f, lambda));  // clamp to valid range.

  float y_t = y_1 + lambda * (y_2 - y_1);

  return std::abs(y_t - cursor.y());
}

void GraphWidget::Timeseries::setFixed(bool value) {
  isFixed_ = value;
}

bool GraphWidget::Timeseries::isFixed() const {
  return isFixed_;
}

const QColor& GraphWidget::Timeseries::color() const {
  return color_;
}

const Qt::PenStyle& GraphWidget::Timeseries::style() const {
  return style_;
}

GraphWidget::Marker::Marker(uint32_t t, const QColor& color) : t(t), color(color) {}

GraphWidget::GraphWidget(QWidget* parent, Qt::WindowFlags f) : QWidget(parent, f), contextMenu_(this) {
  t_current_ = 20;
  margin_ = 5;

  // important to activate:
  setFocusPolicy(Qt::WheelFocus);
  setMouseTracking(true);

  QAction* showLegendAction = new QAction("Show Legend", this);
  showLegendAction->setCheckable(true);
  showLegendAction->setChecked(showLegend_);
  contextMenu_.addAction(showLegendAction);
  connect(showLegendAction, &QAction::triggered,
          [this](bool checked) { this->handleContextAction("legend", checked); });
  contextMenu_.addSeparator();
}

void GraphWidget::reset() {
  visibleTimestamps_ = 20.0f;
  t_current_ = 0;

  for (auto& ts : timeseries_) {
    ts.second->clear();
  }

  markers_.clear();

  update();
}

GraphWidget::Timeseries::Ptr GraphWidget::addTimeseries(const std::string& name, const QColor& color,
                                                        const Qt::PenStyle& style) {
  // currently does not check if duplicate.
  timeseries_[name] = Timeseries::Ptr(new Timeseries(color, style));
  order_.push_back(name);
  visible_[name] = true;

  QAction* showTimeseriesAction = new QAction(QString::fromStdString(name), this);
  showTimeseriesAction->setCheckable(true);
  showTimeseriesAction->setChecked(true);
  contextMenu_.addAction(showTimeseriesAction);
  connect(showTimeseriesAction, &QAction::triggered,
          [this, name](bool checked) { this->handleContextAction(name, checked); });

  return timeseries_[name];
}

GraphWidget::Timeseries::Ptr GraphWidget::getTimeseries(const std::string& name) {
  if (timeseries_.find(name) == timeseries_.end())
    throw std::runtime_error("Timeseries with name '" + name + "' not found!");

  return timeseries_[name];
}

std::map<std::string, GraphWidget::Timeseries::Ptr>& GraphWidget::getAllTimeseries() {
  return timeseries_;
}

void GraphWidget::setTimestamp(uint32_t t) {
  t_current_ = t;

  update();
}

void GraphWidget::mouseMoveEvent(QMouseEvent* event) {
  if (!fixedPosition_) {
    currentCursor_ = event->pos();
    int32_t focusMargin = 2;
    hasFocus_ = (currentCursor_.x() > focusMargin && currentCursor_.y() > focusMargin &&
                 currentCursor_.x() < width() - focusMargin && currentCursor_.y() < height() - focusMargin);
    update();
  }
}

void GraphWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  if (fixedPosition_) {
    fixedPosition_ = false;
  } else {
    fixedPosition_ = true;
    currentCursor_ = event->pos();
  }
}

void GraphWidget::paintEvent(QPaintEvent* event) {
  QPainter painter(this);

  painter.setRenderHint(QPainter::RenderHint::Antialiasing, true);

  painter.fillRect(QRect(0, 0, width(), height()), Qt::white);
  painter.drawRect(QRect(0, 0, width(), height()));

  GraphContext ctx;
  ctx.painter = &painter;
  ctx.t_current = std::max<int32_t>(t_current_, visibleTimestamps_);
  ctx.width = width();
  ctx.height = height();
  ctx.deltaT = float(width() - 2 * margin_) / visibleTimestamps_;
  ctx.margin = margin_;
  ctx.isNearestToCursor = false;
  ctx.cursor = currentCursor_;
  ctx.showLegend = showLegend_;

  // draw grid.
  painter.setPen(QColor::fromRgb(0, 0, 0, 128));
  painter.setFont(QFont("sans", 8));

  int32_t timestep_step = 5;
  while (ctx.deltaT * timestep_step < 45) timestep_step += 5;

  for (int32_t i = ctx.t_current; i >= 0; --i) {
    if (i % timestep_step != 0) continue;
    int32_t idx = ctx.t_current - i;

    int32_t x = ctx.width - idx * ctx.deltaT - margin_;
    int32_t y = 0.5 * ctx.height;

    painter.drawLine(x, 0, x, height());
    QString strTime = QString::number(i);
    QRectF bb = painter.boundingRect(QRect(0, 0, 1, 1), Qt::AlignCenter, strTime);
    float margin = 2;
    QRect textBoundingBox(x - 0.5 * (bb.width() + margin), y - 0.5 * (bb.height() + margin), bb.width() + margin,
                          bb.height() + margin);
    painter.fillRect(textBoundingBox, Qt::white);
    painter.drawText(textBoundingBox, Qt::AlignCenter, strTime);

    if (x < 0) break;
  }

  for (uint32_t i = 0; i < markers_.size(); ++i) {
    if (markers_[i].t > t_current_ || markers_[i].t < t_current_ - visibleTimestamps_) continue;

    int32_t idx = ctx.t_current - markers_[i].t;
    int32_t x = ctx.width - idx * ctx.deltaT - margin_;

    painter.setPen(markers_[i].color);
    painter.drawLine(x, margin_, x, ctx.height - margin_);
  }

  if (hasFocus_ && !fixedPosition_) {
    nearestIndex_ = -1;
    float minDistance = 25.0f;  // min pixels.
    for (int32_t i = order_.size() - 1; i >= 0; --i) {
      if (visible_[order_[i]]) {
        float dist = timeseries_[order_[i]]->distance(ctx, currentCursor_);
        if (dist < minDistance) {
          minDistance = dist;
          nearestIndex_ = i;
        }
      }
    }
  }

  ctx.isNearestToCursor = false;
  for (int32_t i = 0; i < (int32_t)order_.size(); ++i) {
    if (visible_[order_[i]]) {
      ctx.isNearestToCursor = (i == nearestIndex_);
      timeseries_[order_[i]]->draw(ctx);
    }
  }

  if (showLegend_ && nearestIndex_ > -1) {
    QString title = QString::fromStdString(order_[nearestIndex_]);
    const QColor& color = timeseries_[order_[nearestIndex_]]->color();
    const Qt::PenStyle& style = timeseries_[order_[nearestIndex_]]->style();

    QRectF bbox = painter.boundingRect(0, 0, 100, 2, Qt::AlignCenter, title);
    int32_t padding = 5;
    int32_t appearanceWidth = 15;
    bbox.setWidth(std::max<float>(100, bbox.width()));
    QRect legendBox(10, 10, bbox.width() + appearanceWidth + padding + 2 * padding, bbox.height() + 2 * padding);
    QRect legendInnerBox(legendBox.x() + 2 * padding, legendBox.y() + padding, bbox.width() + appearanceWidth,
                         bbox.height());

    painter.setPen(Qt::black);
    painter.setBrush(Qt::NoBrush);
    painter.fillRect(legendBox, Qt::white);
    painter.drawRect(legendBox);
    painter.drawText(legendInnerBox, Qt::AlignRight | Qt::AlignVCenter, title);

    QPen pen(color);
    pen.setStyle(style);
    pen.setWidth(2);

    painter.setPen(pen);
    painter.drawLine(legendBox.x() + padding, legendBox.y() + 0.5 * legendBox.height(),
                     legendBox.x() + appearanceWidth + padding, legendBox.y() + 0.5 * legendBox.height());
  }

  painter.end();
}

void GraphWidget::wheelEvent(QWheelEvent* event) {
  // important: Set wheelfocus in the constructor to allow wheel events.

  if (event->delta() < 0) {
    visibleTimestamps_ *= 1.2;
  } else {
    visibleTimestamps_ *= 0.8;
  }

  update();
  event->accept();
}

void GraphWidget::contextMenuEvent(QContextMenuEvent* event) {
  contextMenu_.exec(event->globalPos());
}

void GraphWidget::handleContextAction(const std::string& target, bool checked) {
  if (target == "legend") {
    showLegend_ = checked;
  } else {
    visible_[target] = checked;
  }
}

void GraphWidget::insertMarker(uint32_t t, const QColor& color) {
  markers_.push_back(Marker(t, color));
}
