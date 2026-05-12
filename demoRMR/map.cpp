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
    if(gridData.empty())
        return;

    QPainter painter(this);

    painter.fillRect(rect(), Qt::black);

    int cellSize = 2;
    double resolution = 0.05;

    if(!distanceField.empty() &&
        distanceField.size() == gridData.size() &&
        distanceField[0].size() == gridData[0].size())
    {
        for(int i = 0; i < static_cast<int>(distanceField.size()); i++)
        {
            for(int j = 0; j < static_cast<int>(distanceField[i].size()); j++)
            {
                double d = distanceField[i][j];

                int gray =
                    std::min(255,
                             static_cast<int>(d * 40));

                painter.fillRect(
                    i * cellSize,
                    j * cellSize,
                    cellSize,
                    cellSize,
                    QColor(gray, gray, gray));
            }
        }
    }

    painter.setPen(Qt::white);
    painter.setBrush(Qt::white);

    for(int i = 0; i < static_cast<int>(gridData.size()); i++)
    {
        for(int j = 0; j < static_cast<int>(gridData[i].size()); j++)
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
        int grid_i =
            static_cast<int>(p.x / resolution) +
            static_cast<int>(gridData.size()) / 2;

        int grid_j =
            static_cast<int>(p.y / resolution) +
            static_cast<int>(gridData[0].size()) / 2;

        if(grid_i < 0 || grid_i >= static_cast<int>(gridData.size()) ||
            grid_j < 0 || grid_j >= static_cast<int>(gridData[0].size()))
        {
            continue;
        }

        int px = grid_i * cellSize;
        int py = grid_j * cellSize;

        painter.drawEllipse(QPoint(px, py), 2, 2);
    }
}

void MapWidget::setDistanceField(
    const std::vector<std::vector<double>>& df)
{
    distanceField = df;
    update();
}
