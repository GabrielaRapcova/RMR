#ifndef MAP_H
#define MAP_H

#pragma once

#include <QWidget>
#include <vector>
#include "robot.h"

class MapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MapWidget(QWidget *parent = nullptr);

    void setGrid(const std::vector<std::vector<int>> &grid);

    void setParticles(const std::vector<robot::Particle>& p);
    void setDistanceField(const std::vector<std::vector<double>>& df);
    void setEstimatedPose(
        double x,
        double y,
        double fi);

protected:
    void paintEvent(QPaintEvent *) override;

private:

    struct Particle
    {
        double x;
        double y;
        double fi;
        double weight;
    };
    std::vector<std::vector<int>> gridData;
std::vector<std::vector<double>> distanceField;
    std::vector<robot::Particle> particles;
double estimatedX;
double estimatedY;
double estimatedFi;
};

#endif // MAP_H
