class Vec3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other_vect):
        vector = Vec3D(None, None, None)
        vector.x = self.x + other_vect.x
        vector.y = self.y + other_vect.y
        vector.z = self.z + other_vect.z

        return vector

    def __sub__(self, other_vect):
        vector = Vec3D(None, None, None)
        vector.x = self.x - other_vect.x
        vector.y = self.y - other_vect.y
        vector.z = self.z - other_vect.z

        return vector

    def __mul__(self, scalar):
        vector = Vec3D(None, None, None)
        vector.x = scalar * self.x
        vector.y = scalar * self.y
        vector.z = scalar * self.z

        return vector

    def __rmul__(self, scalar):
        vector = Vec3D(None, None, None)
        vector.x = scalar * self.x
        vector.y = scalar * self.y
        vector.z = scalar * self.z

        return vector

    def __eq__(self, vector):
        return (self.x == vector.x and
                self.y == vector.y and
                self.z == vector.z)
