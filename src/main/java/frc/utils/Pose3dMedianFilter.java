
package frc.utils;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class Pose3dMedianFilter {
    private final MedianFilter m_xFilter;
    private final MedianFilter m_yFilter;
    private final MedianFilter m_zFilter;
    private final MedianFilter m_rotXFilter;
    private final MedianFilter m_rotYFilter;
    private final MedianFilter m_rotZFilter;

    public Pose3dMedianFilter(int windowSize) {
        m_xFilter = new MedianFilter(windowSize);
        m_yFilter = new MedianFilter(windowSize);
        m_zFilter = new MedianFilter(windowSize);
        m_rotXFilter = new MedianFilter(windowSize);
        m_rotYFilter = new MedianFilter(windowSize);
        m_rotZFilter = new MedianFilter(windowSize);
    }

    public Pose3d calculate(Pose3d pose) {
        Translation3d translation = pose.getTranslation();
        Rotation3d rotation = pose.getRotation();
        return new Pose3d(
            m_xFilter.calculate(translation.getX()),
            m_yFilter.calculate(translation.getY()),
            m_zFilter.calculate(translation.getZ()),
            new Rotation3d(
                m_rotXFilter.calculate(rotation.getX()),
                m_rotYFilter.calculate(rotation.getY()),
                m_rotZFilter.calculate(rotation.getZ())
            )
        );
    }

    public void reset() {
        m_xFilter.reset();
        m_yFilter.reset();
        m_zFilter.reset();
        m_rotXFilter.reset();
        m_rotYFilter.reset();
        m_rotZFilter.reset();
    }

    public Pose3d lastValue() {
        return new Pose3d(
            m_xFilter.lastValue(),
            m_yFilter.lastValue(),
            m_zFilter.lastValue(),
            new Rotation3d(
                m_rotXFilter.lastValue(),
                m_rotYFilter.lastValue(),
                m_rotZFilter.lastValue()
            )
        );
    }


}
