#include "map.h"
#include <QPainter>

MapWidget::MapWidget(QWidget *parent)
    : QWidget(parent)
{
    resize(600,600);
}

void MapWidget::setGrid(const std::vector<std::vector<int>> &grid)
{
    gridData = grid;
    update();
}
void MapWidget::setParticles(const std::vector<robot::Particle>& p)
{
    particles = p;
    update();
}
void MapWidget::setEstimatedPose(
    double x,
    double y,
    double fi)
{
    estimatedX = x;
    estimatedY = y;
    estimatedFi = fi;

    update();
}
void MapWidget::paintEvent(QPaintEvent *)
{
    // Check both are not empty
    if(distanceField.empty() || gridData.empty() || gridData[0].empty())
        return;

    QPainter painter(this);

    painter.fillRect(rect(), Qt::black);

    int cellSize = 2;

    for(int i=0; i<distanceField.size(); i++)
    {
        for(int j=0; j<distanceField[i].size(); j++)
        {
            double d = distanceField[i][j];

            int gray =
                std::min(255,
                         (int)(d * 40));

            painter.fillRect(
                i * cellSize,
                j * cellSize,
                cellSize,
                cellSize,
                QColor(gray, gray, gray));
        }
    }

    painter.setPen(Qt::white);

    for(int i=0; i<gridData.size(); i++)
    {
        for(int j=0; j<gridData[i].size(); j++)
        {
            if(gridData[i][j] == 1)
            {
                painter.drawRect(
                    i * cellSize,
                    j * cellSize,
                    cellSize,
                    cellSize);
            }
        }
    }

    painter.setPen(Qt::red);
    painter.setBrush(Qt::red);

    for(const auto& p : particles)
    {
        int px = (p.x / 0.05) + gridData.size()/2;
        int py = (p.y / 0.05) + gridData[0].size()/2;

        px *= cellSize;
        py *= cellSize;

        painter.drawEllipse(QPoint(px, py), 2, 2);
    }

    painter.setPen(Qt::green);
    painter.setBrush(Qt::green);

    int ex =
        (estimatedX / 0.05 +
         gridData.size()/2) * cellSize;

    int ey =
        (estimatedY / 0.05 +
         gridData[0].size()/2) * cellSize;

    painter.drawEllipse(
        QPoint(ex, ey),
        5,
        5);

    int lineLength = 15;

    int endX =
        ex +
        lineLength *
            cos(estimatedFi);

    int endY =
        ey +
        lineLength *
            sin(estimatedFi);

    painter.drawLine(
        ex,
        ey,
        endX,
        endY);
}
void MapWidget::setDistanceField(
    const std::vector<std::vector<double>>& df)
{
    distanceField = df;
    update();
}
