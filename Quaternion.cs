using System;

namespace Numbers
{
    public class Quaternion
    {
        public Quaternion(float w_, float xi, float yj, float zk)
        {
            w = w_;
            x = xi;
            y = yj;
            z = zk;
        }
        private float w;
        public float W
        {
            set
            { w = value; }
            get
            { return w; }
        }
        private float x;
        public float X
        {
            set { x = value; }
            get { return x; }
        }
        private float y;
        public float Y
        {
            set { y = value; }
            get { return y; }
        }
        private float z;
        public float Z
        {
            set { z = value; }
            get { return z; }
        }
        /// <summary>
        /// Euclidean norm
        /// </summary>
        public float Norm
        {
            get { return (float)Math.Sqrt(w * w + x * x + y * y + z * z); }
        }
        /// <summary>
        /// Conjugate
        /// </summary>
        public Quaternion Conj
        {
            get { return new Quaternion(w, -x, -y, -z); }
        }
        public static Quaternion operator +(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.W + q2.W, q1.X + q2.X, q1.Y + q2.Y, q1.Z + q2.Z);
        }
        public static Quaternion operator -(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.W - q2.W, q1.X - q2.X, q1.Y - q2.Y, q1.Z - q2.Z);
        }
        /// <summary>
        /// product of two quaterions
        /// </summary>
        /// <param name="q1">Quaternion1
        /// <param name="q2">Quaternion2
        /// <returns>Quaternion1*Quaternion2</returns>
        public static Quaternion operator *(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
                , q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
                , q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z
                , q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x);
        }
        public static Quaternion operator *(float f, Quaternion q)
        {
            return new Quaternion(f * q.w, f * q.x, f * q.y, f * q.z);
        }
        public static Quaternion operator *(Quaternion q, float f)
        {
            return new Quaternion(f * q.w, f * q.x, f * q.y, f * q.z);
        }
        public static Quaternion operator /(Quaternion q, float f)
        {
            if (f == 0.0f) { throw new DivideByZeroException(); }
            return new Quaternion(1 / f * q.w, 1 / f * q.x, 1 / f * q.y, 1 / f * q.z);
        }
        public static Quaternion operator /(float f, Quaternion q)
        {
            if (q.Norm == 0.0f) { throw new DivideByZeroException(); }
            return f / (q.Norm * q.Norm) * q.Conj;
        }
        public static Quaternion operator /(Quaternion q1, Quaternion q2)
        {
            return q1 * q2.Conj / (q2.Norm * q2.Norm);
        }
        public static bool operator ==(Quaternion q1, Quaternion q2)
        {
            if (Math.Abs(q1.w - q2.w) < 0.00001f
                && Math.Abs(q1.x - q2.x) < 0.00001f
                && Math.Abs(q1.y - q2.y) < 0.00001f
                && Math.Abs(q1.z - q2.z) < 0.00001f)
            {
                return true;
            }
            return false;
        }
        public static bool operator !=(Quaternion q1, Quaternion q2)
        {
            if (q1.w == q2.w && q1.x == q2.x && q1.y == q2.y && q1.z == q2.z)
            {
                return false;
            }
            return true;
        }
        public override bool Equals(object obj)
        {
            Quaternion q = obj as Quaternion;
            return q == this;
        }
        public override int GetHashCode()
        {
            return this.w.GetHashCode() ^ (this.x.GetHashCode() * this.y.GetHashCode() * this.z.GetHashCode());
        }
        public float[] Rotate(float x1, float y1, float z1)
        {
            Quaternion q = new Quaternion(0.0f, x1, y1, z1);
            Quaternion r = this * q * this.Conj;
            return new float[3] { r.X, r.Y, r.Z };
        }
    }
}