#ifndef MAP_H
#define MAP_H

#pragma once

#include <QWidget>
#include <vector>

class MapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MapWidget(QWidget *parent = nullptr);

    void setGrid(const std::vector<std::vector<int>> &grid);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    std::vector<std::vector<int>> gridData;
};

#endif // MAP_H
