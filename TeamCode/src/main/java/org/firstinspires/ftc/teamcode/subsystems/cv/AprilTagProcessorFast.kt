package org.firstinspires.ftc.teamcode.subsystems.cv

import android.graphics.Canvas
import com.qualcomm.robotcore.util.MovingStatistics
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.opencv.calib3d.Calib3d
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point
import org.opencv.core.Point3
import org.openftc.apriltag.AprilTagDetectorJNI
import org.openftc.apriltag.ApriltagDetectionJNI
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.round

class AprilTagProcessorFast(var fx: Double, var fy: Double, var cx: Double, var cy: Double): AprilTagProcessor() {

    var tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary()

    var grey = Mat()
    var detections = arrayListOf<AprilTagDetection>()

    val decimationSync = Mutex()
    var needToSetDecimation = false
    var decimation = 3f

    var detectionsUpdate: ArrayList<AprilTagDetection>? = null
    val detectionsUpdateSync = Mutex()

    lateinit var cameraMatrix: Mat
    lateinit var canvasAnnotator: AprilTagCanvasAnnotator

    var nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(TagFamily.TAG_36h11.toString(), decimation, 6)

    var poseSolver = PoseSolver.OPENCV_ITERATIVE
    var solveTime = MovingStatistics(50)

    var outputUnitsLength = DistanceUnit.INCH
    var outputUnitsAngle = AngleUnit.RADIANS
    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        constructMatrix()
        canvasAnnotator = AprilTagCanvasAnnotator(cameraMatrix)

    }

    protected fun finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0L) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr)
            nativeApriltagPtr = 0
        } else {
            println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL")
        }
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any {
        runBlocking {
            decimationSync.withLock {
                if(needToSetDecimation){
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation)
                    needToSetDecimation = true
                }
            }
        }

        detections = runAprilTagDetectorForMultipleTagSizes(captureTimeNanos)

        runBlocking {
            detectionsUpdateSync.withLock {
                detectionsUpdate = detections
            }
        }

        return detections
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        TODO("Not yet implemented")
    }

    override fun setDecimation(decimation: Float) {
        runBlocking {
            decimationSync.withLock {
                this@AprilTagProcessorFast.decimation = decimation
                needToSetDecimation = true
            }
        }
    }

    override fun setPoseSolver(poseSolver: PoseSolver?) {
        TODO("Not yet implemented")
    }

    override fun getPerTagAvgPoseSolveTime(): Int {
        return round(solveTime.mean).toInt()
    }

    override fun getDetections(): ArrayList<AprilTagDetection> {
        return detections
    }

    override fun getFreshDetections(): ArrayList<AprilTagDetection> {
        runBlocking {
            detectionsUpdateSync.withLock {
                val ret = detectionsUpdate
                detectionsUpdate = null
                return@runBlocking ret
            }
        }
        return detections
    }

    fun constructMatrix() {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //
        cameraMatrix = Mat(3, 3, CvType.CV_32FC1)
        cameraMatrix.put(0, 0, fx)
        cameraMatrix.put(0, 1, 0.0)
        cameraMatrix.put(0, 2, cx)
        cameraMatrix.put(1, 0, 0.0)
        cameraMatrix.put(1, 1, fy)
        cameraMatrix.put(1, 2, cy)
        cameraMatrix.put(2, 0, 0.0)
        cameraMatrix.put(2, 1, 0.0)
        cameraMatrix.put(2, 2, 1.0)
    }

    fun runAprilTagDetectorForMultipleTagSizes(captureTimeNanos: Long): ArrayList<AprilTagDetection> {
        val ptrDetectionArray = AprilTagDetectorJNI.runApriltagDetector(
            nativeApriltagPtr,
            grey.dataAddr(),
            grey.width(),
            grey.height()
        )
        if (ptrDetectionArray != 0L) {
            val detectionPointers = ApriltagDetectionJNI.getDetectionPointers(ptrDetectionArray)
            val detections = ArrayList<AprilTagDetection>(detectionPointers.size)
            for (ptrDetection in detectionPointers) {
                val metadata = tagLibrary.lookupTag(ApriltagDetectionJNI.getId(ptrDetection))
                val corners = ApriltagDetectionJNI.getCorners(ptrDetection)
                val cornerPts = arrayOfNulls<Point>(4)
                for (p in 0..3) {
                    cornerPts[p] = Point(corners[p][0], corners[p][1])
                }
                var rawPose: AprilTagPoseRaw?
                if (metadata != null) {
                    val solver = poseSolver // snapshot, can change
                    val startSolveTime = System.currentTimeMillis()
                    if (solver == PoseSolver.APRILTAG_BUILTIN) {
                        val pose = ApriltagDetectionJNI.getPoseEstimate(
                            ptrDetection,
                            outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                            fx, fy, cx, cy
                        )

                        // Build rotation matrix
                        val rotMtxVals = FloatArray(3 * 3)
                        for (i in 0..8) {
                            rotMtxVals[i] = pose[3 + i].toFloat()
                        }
                        rawPose = AprilTagPoseRaw(
                            pose[0], pose[1], pose[2],  // x y z
                            GeneralMatrixF(3, 3, rotMtxVals)
                        ) // R
                    } else {
                        val opencvPose = poseFromTrapezoid(
                            cornerPts,
                            cameraMatrix,
                            outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                            0
                        )

                        // Build rotation matrix
                        val R = Mat(3, 3, CvType.CV_32F)
                        Calib3d.Rodrigues(opencvPose.rvec, R)
                        val tmp2 = FloatArray(9)
                        R[0, 0, tmp2]
                        rawPose = AprilTagPoseRaw(
                            opencvPose.tvec[0, 0][0],  // x
                            opencvPose.tvec[1, 0][0],  // y
                            opencvPose.tvec[2, 0][0],  // z
                            GeneralMatrixF(3, 3, tmp2)
                        ) // R
                    }
                    val endSolveTime = System.currentTimeMillis()
                    solveTime.add((endSolveTime - startSolveTime).toDouble())
                } else {
                    // We don't know anything about the tag size so we can't solve the pose
                    rawPose = null
                }
                val ftcPose: AprilTagPoseFtc? = if (rawPose != null) {
                    val rot = Orientation.getOrientation(
                        rawPose.R,
                        AxesReference.INTRINSIC,
                        AxesOrder.YXZ,
                        outputUnitsAngle
                    )
                    AprilTagPoseFtc(
                        rawPose.x,  // x   NB: These are *intentionally* not matched directly;
                        rawPose.z,  // y       this is the mapping between the AprilTag coordinate
                        -rawPose.y,  // z       system and the FTC coordinate system
                        -rot.firstAngle.toDouble(),  // yaw
                        rot.secondAngle.toDouble(),  // pitch
                        rot.thirdAngle.toDouble(),  // roll
                        hypot(rawPose.x, rawPose.z),  // range
                        outputUnitsAngle.fromUnit(
                            AngleUnit.RADIANS,
                            atan2(-rawPose.x, rawPose.z)
                        ),  // bearing
                        outputUnitsAngle.fromUnit(
                            AngleUnit.RADIANS,
                            atan2(-rawPose.y, rawPose.z)
                        )
                    ) // elevation
                } else {
                    null
                }
                val center = ApriltagDetectionJNI.getCenterpoint(ptrDetection)
                detections.add(
                    AprilTagDetection(
                        ApriltagDetectionJNI.getId(ptrDetection),
                        ApriltagDetectionJNI.getHamming(ptrDetection),
                        ApriltagDetectionJNI.getDecisionMargin(ptrDetection),
                        Point(center[0], center[1]),
                        cornerPts,
                        metadata,
                        ftcPose,
                        rawPose,
                        captureTimeNanos
                    )
                )
            }
            ApriltagDetectionJNI.freeDetectionList(ptrDetectionArray)
            return detections
        }
        return ArrayList()
    }


    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsize the original length of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    fun poseFromTrapezoid(
        points: Array<Point?>,
        cameraMatrix: Mat?,
        tagsize: Double,
        solveMethod: Int
    ): Pose {
        // The actual 2d points of the tag detected in the image
        val points2d = MatOfPoint2f(*points)

        // The 3d points of the tag in an 'ideal projection'
        val arrayPoints3d = arrayOfNulls<Point3>(4)
        arrayPoints3d[0] = Point3(-tagsize / 2, tagsize / 2, 0.0)
        arrayPoints3d[1] = Point3(tagsize / 2, tagsize / 2, 0.0)
        arrayPoints3d[2] = Point3(tagsize / 2, -tagsize / 2, 0.0)
        arrayPoints3d[3] = Point3(-tagsize / 2, -tagsize / 2, 0.0)
        val points3d = MatOfPoint3f(*arrayPoints3d)

        // Using this information, actually solve for pose
        val pose = Pose()
        Calib3d.solvePnP(
            points3d,
            points2d,
            cameraMatrix,
            MatOfDouble(),
            pose.rvec,
            pose.tvec,
            false,
            solveMethod
        )
        return pose
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose {
        var rvec: Mat
        var tvec: Mat

        constructor() {
            rvec = Mat(3, 1, CvType.CV_32F)
            tvec = Mat(3, 1, CvType.CV_32F)
        }

        constructor(rvec: Mat, tvec: Mat) {
            this.rvec = rvec
            this.tvec = tvec
        }
    }
}