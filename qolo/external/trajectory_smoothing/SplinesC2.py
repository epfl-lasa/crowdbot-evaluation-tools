import SplinesC1
from SplinesC0 import InvalidData


class SplineC2:
    def __init__(self, degree, intervals):
        if degree == 2:
            raise InvalidData(SplinesC1.SplineC1.NONE_EXISTING_SPLINE_MESSAGE)

        self.splineC1 = SplinesC1.SplineC1(degree, intervals)
        self.deBoor_points = []
        self.control_points = []

        self.to_be_drawn = False
        self.control_polygon_to_be_drawn = False
        self.are_points_calculated = False
        self.control_polygon_Bezier_to_be_drawn = False
        self.control_polygon_deBoor_to_be_drawn = False

    def append_deBoor_point(self, point):
        if len(self.deBoor_points) == (
                2 * (-1 + self.splineC1.splineC0._degree) +
                len(self.splineC1.splineC0.intervals) - 1):
            raise IndexError

        self.deBoor_points.append(point)

    def _calculate_splineC1_point(self, movement=None):
        degree = self.splineC1.splineC0._degree

        if not movement:
            for index in range(0, degree - 1):
                self.splineC1.append_deBoor_point(
                    self.deBoor_points[index])

        intervals = self.splineC1.splineC0.intervals
        intervals_count = len(intervals)
        counter_splineC1 = degree - 1

        for index in range(0, intervals_count):
            if index == 0:
                previous_interval = 0
                next_interval = intervals[index + 1]
            elif index == intervals_count - 1:
                previous_interval = intervals[index - 1]
                next_interval = 0
            else:
                previous_interval, next_interval = (intervals[index - 1],
                                                    intervals[index + 1])

            denominator = (previous_interval + intervals[index] +
                           next_interval)

            if index > 0:
                numerator = ((intervals[index] + next_interval) *
                             self.deBoor_points[index + degree - 2] +
                             previous_interval *
                             self.deBoor_points[index + degree - 1])

                calculated_point = numerator * (1 / float(denominator))

                if not movement:
                    self.splineC1.append_deBoor_point(calculated_point)
                else:
                    self.splineC1.replace_point(counter_splineC1,
                                                calculated_point)
                    counter_splineC1 += 1

            if index < intervals_count - 1:
                numerator = (next_interval *
                             self.deBoor_points[index + degree - 2] +
                             (intervals[index] + previous_interval) *
                             self.deBoor_points[index + degree - 1])

                calculated_point = numerator * (1 / float(denominator))

                if not movement:
                    self.splineC1.append_deBoor_point(calculated_point)
                else:
                    self.splineC1.replace_point(counter_splineC1,
                                                calculated_point)
                    counter_splineC1 += 1

        if not movement:
            for index in range(degree - 1, 0, -1):
                self.splineC1.append_deBoor_point(self.deBoor_points[
                    len(self.deBoor_points) - index])

    def replace_point(self, index, point):
        self.deBoor_points[index] = point
        self._calculate_splineC1_point(True)

        degree = self.splineC1.splineC0._degree
        partial_curves_count = len(self.splineC1.splineC0.partial_curves)

        if index < degree - 1:
            self.splineC1.replace_point(index, point)

        if index > len(self.deBoor_points) - degree:
            self.splineC1.replace_point(index + partial_curves_count - 1,
                                        point)

    def draw(self):
        if len(self.deBoor_points) < (
                2 * (-1 + self.splineC1.splineC0._degree) +
                len(self.splineC1.splineC0.intervals) - 1):
            raise InvalidData(
                self.splineC1.INCORRECT_COUNT_DEBOOR_POINTS_MESSAGE)

        if not self.are_points_calculated:
            self._calculate_splineC1_point()
            self.are_points_calculated = True

        draw_points = self.splineC1.draw()
        self.control_points = self.splineC1.control_points

        return draw_points
