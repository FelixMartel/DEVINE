#! /usr/bin/env python2
import rosunit
from devine_tests.segmentation.rate.test_segmentation_rate import TestSegmentationRate
from devine_tests.segmentation.image_quality.test_segmentation_quality import TestSegmentationQuality

TEST_PACKAGE = "tests"

if __name__ == "__main__":
    rosunit.unitrun(TEST_PACKAGE, "Segmentation Rate", TestSegmentationQuality)
    rosunit.unitrun(TEST_PACKAGE, "Segmentation Rate", TestSegmentationRate)
