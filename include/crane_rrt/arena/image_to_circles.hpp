#ifndef IMAGE_TO_SPHERES_HPP
#define IMAGE_TO_SPHERES_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>

namespace arena {

class ImageToCircles {
public:
    struct Circle {
        cv::Point2f centerNorm;  // normalized [0.0, 1.0]
        float radiusNorm;
    };

    struct CircleMetric {
        cv::Point2f centerMeters;
        float radiusMeters;
    };

    ImageToCircles(double minRadiusPx = 5.0, double whitenessThreshold = 250.0)
        : mMinRadiusPx(minRadiusPx),
          mWhitenessThreshold(whitenessThreshold),
          mPixelSizeCm(1.0) {}

    bool loadImage(const std::string& path, double pixelSizeCm) {
        mImage = cv::imread(path, cv::IMREAD_COLOR);
        if (mImage.empty()) {
            std::cerr << "Failed to load image: " << path << std::endl;
            return false;
        }

        mPixelSizeCm = pixelSizeCm;
        mImageSize = mImage.size();
        computeForegroundMask();
        return true;
    }

    void process() {
        if (mImage.empty() || mForegroundMask.empty()) {
            std::cerr << "No valid image loaded.\n";
            return;
        }
        fillWithCircles();
        computeConnectedComponents();
    }

    const std::vector<Circle>& getCircles() const {
        return mCircles;
    }

    std::vector<CircleMetric> getCirclesInMeters() const {
        std::vector<CircleMetric> metrics;
        const double pxToMeter = mPixelSizeCm / 100.0;

        for (const auto& c : mCircles) {
            float x_m = c.centerNorm.x * mImageSize.width * pxToMeter;
            float y_m = c.centerNorm.y * mImageSize.height * pxToMeter;
            float r_m = c.radiusNorm * std::min(mImageSize.width, mImageSize.height) * pxToMeter;
            metrics.push_back({ cv::Point2f(x_m, y_m), r_m });
        }

        return metrics;
    }

    std::unordered_map<int, std::vector<Circle>> getLabeledCircles() const {
        std::unordered_map<int, std::vector<Circle>> labelToCircles;

        if (mLabelImage.empty()) {
            std::cerr << "Warning: Label image not available. Run `process()` first.\n";
            return labelToCircles;
        }

        for (const auto& c : mCircles) {
            int xPx = static_cast<int>(c.centerNorm.x * mImageSize.width);
            int yPx = static_cast<int>(c.centerNorm.y * mImageSize.height);
            if (xPx < 0 || xPx >= mLabelImage.cols || yPx < 0 || yPx >= mLabelImage.rows)
                continue;

            int label = mLabelImage.at<int>(yPx, xPx);
            if (label > 0) {  // skip background
                labelToCircles[label].push_back(c);
            }
        }

        return labelToCircles;
    }

    cv::Size getImageSize() const { return mImageSize; }

private:
    void computeForegroundMask() {
        cv::Mat gray;
        cv::cvtColor(mImage, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, mForegroundMask, mWhitenessThreshold, 255, cv::THRESH_BINARY_INV);
    }

    void computeConnectedComponents() {
        cv::connectedComponents(mForegroundMask, mLabelImage, 8, CV_32S);
    }

    void fillWithCircles() {
        mCircles.clear();

        cv::Mat mask;
        mForegroundMask.convertTo(mask, CV_8UC1);
        mask /= 255;  // binary mask 0/1

        while (true) {
            cv::Mat dist;
            cv::distanceTransform(mask, dist, cv::DIST_L2, 5);

            double maxVal;
            cv::Point maxLoc;
            cv::minMaxLoc(dist, nullptr, &maxVal, nullptr, &maxLoc);

            if (maxVal < mMinRadiusPx)
                break;

            float radiusPx = static_cast<float>(maxVal);
            float xNorm = (static_cast<float>(maxLoc.x) / mImageSize.width);
            float yNorm = (static_cast<float>(maxLoc.y) / mImageSize.height);
            float rNorm = radiusPx / std::min(mImageSize.width, mImageSize.height);

            mCircles.push_back({ cv::Point2f(xNorm, yNorm), rNorm });

            cv::circle(mask, maxLoc, static_cast<int>(radiusPx), 0, -1);
        }
    }

    cv::Mat mImage;
    cv::Mat mForegroundMask;
    cv::Mat mLabelImage;
    std::vector<Circle> mCircles;

    double mMinRadiusPx;
    double mWhitenessThreshold;
    double mPixelSizeCm;
    cv::Size mImageSize;
};

} // namespace arena

#endif // IMAGE_TO_SPHERES_HPP
