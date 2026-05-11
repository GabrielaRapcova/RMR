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

void MapWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    painter.fillRect(rect(), Qt::black);

    int cellSize = 2;

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

}
