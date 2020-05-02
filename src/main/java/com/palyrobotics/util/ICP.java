package com.palyrobotics.util;

public class ICP {

    private PointCloud reference; // The field
    private long timeout; // Timeout for each call of doICP in nano seconds

    public Transform doICP(PointCloud lidarOutput) {
        return doICP(lidarOutput, new Transform());
    }

    public Transform doICP(PointCloud lidarOutput, Transform transform) {
        // https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/Constants.java#L56
        long startingTime = System.nanoTime();
        double lastMeanDist = Double.POSITIVE_INFINITY;

        while (startingTime - System.nanoTime() <= timeout) {
            final Transform transInv = transform.inverse();

            final double threshold = lastMeanDist;
            double sumDists = 0;

            /// get pairs of corresponding points
            double SumXa = 0, SumXb = 0, SumYa = 0, SumYb = 0;
            double Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
            int N = 0;
            for (Point p : lidarOutput) {
                Point p2 = transInv.apply(p);
                Point rp = reference.getClosestPoint(p2);
                double dist = p2.distanceTo(rp);
                sumDists += dist;
                if (dist > threshold) continue;
                N++;

                // Compute the terms:
                SumXa += p.x;
                SumYa += p.y;

                SumXb += rp.x;
                SumYb += rp.y;

                Sxx += p.x * rp.x;
                Sxy += p.x * rp.y;
                Syx += p.y * rp.x;
                Syy += p.y * rp.y;
            }

            lastMeanDist = sumDists / N;

            /// calculate the new transform
            // code based on http://mrpt.ual.es/reference/devel/se2__l2_8cpp_source.html#l00158
            if (N == 0) throw new RuntimeException("ICP: no matching points"); // TODO: handle this better, or avoid it
            final double N_inv = 1.0 / N;

            final double mean_x_a = SumXa * N_inv;
            final double mean_y_a = SumYa * N_inv;
            final double mean_x_b = SumXb * N_inv;
            final double mean_y_b = SumYb * N_inv;

            // Auxiliary variables Ax,Ay:
            final double Ax = N * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
            final double Ay = SumXa * SumYb + N * (Syx - Sxy) - SumXb * SumYa;

            final double theta = (Ax == 0 && Ay == 0) ? 0.0 : Math.atan2(Ay, Ax);

            final double ccos = Math.cos(theta);
            final double csin = Math.sin(theta);

            final double tx = mean_x_a - mean_x_b * ccos + mean_y_b * csin;
            final double ty = mean_y_a - mean_x_b * csin - mean_y_b * ccos;

            Transform prevTrans = transform;
            transform = new Transform(theta, tx, ty);
            if (isConverged(prevTrans, transform)) {
                break;
            }
        }
        return transform;
    }

    private boolean isConverged(Transform prev, Transform cur) {
        return Math.abs(prev.theta - cur.theta) < 0.01 && // TODO: magic numbers...
                Math.abs(prev.tx - cur.tx) < 0.01 &&
                Math.abs(prev.ty - cur.ty) < 0.01;
    }
}
