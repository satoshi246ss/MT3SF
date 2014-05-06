﻿using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using OpenCvSharp;

namespace PictureViewer
{
    [StructLayout(LayoutKind.Sequential)]
    public struct KV_DATA
    {
        public Byte x1, x0, x3, x2;   //unsigned char  xpos data req
        public Byte y1, y0, y3, y2;   //unsigned char  ypos status
        public Byte xx1, xx0, xx3, xx2;   //unsigned char  x2pos 
        public Byte yy1, yy0, yy3, yy2;   //unsigned char  y2pos 
        public Byte v11, v10, v13, v12;   //unsigned short x1v, y1v 
        public Byte v21, v20, v23, v22;   //unsigned short x2v, y2v 
        //public UInt16 x1v, y1v, x2v, y2v;    //unsigned short v
    }

    public class Udp_kv : ICloneable
    {
        //状態を表す定数
        const short mmWest = 0;
        const short mmEast = 1;
        const short azErr = -1;
        const short altErr = -2;
        //上の2つ状態を保持します
        public short mt3mode;

        public KV_DATA kd = new KV_DATA();
        public Int32 xpos, ypos, x2pos, y2pos;
        public Int32 x1v, y1v, x2v, y2v;
        public UInt16 kv_status, data_request;
        public double vaz2_kv, valt2_kv, az2_c, alt2_c;
        public string binStr_status, binStr_request;

        CvKalman kv_kalman = Cv.CreateKalman(4, 2);
        int kalman_id = 0;
        public double kvaz, kvalt;
        // 観測値(kalman)
        CvMat measurement = new CvMat(2, 1, MatrixType.F32C1);
        CvMat correction;
        CvMat prediction;

        public int mt3state_move = 0;//     = (G_kv_status  & (1<<12)); //導入中フラグ
        public int mt3state_truck = 0;//     = (G_kv_status  & (1<<13)); //追尾中フラグ
        public int mt3state_night = 0;//     = (G_kv_status  & (1<<14)); //夜間フラグ
        public int mt3state_center = 0;// (data_request & (1 << 2)); //センタリング中フラグ

        public int mt3state_move_pre = 0;//     = (G_kv_status  & (1<<12)); //導入中フラグ
        public int mt3state_truck_pre = 0;//     = (G_kv_status  & (1<<13)); //追尾中フラグ
        public int mt3state_night_pre = 0;//     = (G_kv_status  & (1<<14)); //夜間フラグ
        public int mt3state_center_pre = 0;// (data_request & (1 << 2)); //センタリング中フラグ

        public int kalman_init_flag = 0;  // 1:Form1のカルマンフィルタをリセット

        /// <summary>
        /// udp dataを取り込む。
        /// </summary>
        /// <remarks>
        /// Set save dir name
        /// </remarks>
        public void set_udp_kv_data(byte[] bytes)
        {
            GCHandle gch = GCHandle.Alloc(bytes, GCHandleType.Pinned);
            kd = (KV_DATA)Marshal.PtrToStructure(gch.AddrOfPinnedObject(), typeof(KV_DATA));
            gch.Free();
        }

        /// <summary>
        /// MT3 dataの計算
        /// </summary>
        /// <remarks>
        /// KV_DATA -> az,alt etcに変換
        /// </remarks>
        public void cal_mt3()
        {
            x2pos = (kd.xx2 << 16) + (kd.xx1 << 8) + kd.xx0; // <<16 ->256*256  <<8 ->256
            y2pos = (kd.yy2 << 16) + (kd.yy1 << 8) + kd.yy0; // <<16 ->256*256  <<8 ->256
            x2v = ((kd.v21 << 8) + kd.v20) << 6;
            y2v = ((kd.v23 << 8) + kd.v22) << 6;
            kv_status = (UInt16)((kd.y3 << 8) + kd.y2);      //KV1000 DM503
            data_request = (UInt16)((kd.x3 << 8) + kd.x2);   //KV1000 DM499
            binStr_status = Convert.ToString(kv_status, 2);
            binStr_request = Convert.ToString(data_request, 2);
            Pos2AzAlt2();

            if ((int)(kv_status & (1 << 10)) != 0) vaz2_kv = -x2v / 1000.0;
            else vaz2_kv = +x2v / 1000.0;
            if ((int)(kv_status & (1 << 11)) != 0)
            { //mr107:Y2モータ回転方向
                if (mt3mode == mmEast) valt2_kv = -y2v / 1000.0;
                else valt2_kv = y2v / 1000.0;
            }
            else
            {
                if (mt3mode == mmEast) valt2_kv = y2v / 1000.0;
                else valt2_kv = -y2v / 1000.0;
            }

            mt3state_move_pre = mt3state_move;
            mt3state_truck_pre = mt3state_truck;
            mt3state_center_pre = mt3state_center;
            mt3state_night_pre = mt3state_night;

            mt3state_move = (kv_status & (1 << 12)); //導入中フラグ
            mt3state_truck = (kv_status & (1 << 13)); //追尾中フラグ
            mt3state_night = (kv_status & (1 << 14)); //夜間フラグ
            mt3state_center = (data_request & (1 << 2)); //センタリング中フラグ

            // truck開始時
            if (mt3state_truck_pre == 0 && mt3state_truck != 0) kalman_init_flag=1;

            // センタリング中 完了時
            if (mt3state_center_pre != 0 && mt3state_center == 0) kalman_init_flag = 1;

            kalman_update();
        }

        /// <summary>
        /// MT3 Pos => Az,Alt
        /// </summary>
        /// <remarks>
        /// Pos => Az,Alt　変換
        /// Error code : *mode
        ///  -1 ;  Az  out of range
        ///  -2 ;  Alt out of range
        /// </remarks>
        public void Pos2AzAlt2()
        {
            // Normal
            if (x2pos >= 0 && x2pos < 360000)
            {
                if (y2pos >= 0 && y2pos <= 180000)
                {
                    mt3mode = mmWest; //0
                    az2_c = (90000 + x2pos) / 1000.0;
                    alt2_c = (-90000 + y2pos) / 1000.0;
                }
                else
                    if (y2pos >= 180000 && y2pos < 360000)
                    {
                        mt3mode = mmEast; //1
                        az2_c = (270000 + x2pos) / 1000.0;
                        alt2_c = (270000 - y2pos) / 1000.0;
                    }
                    else
                    {
                        mt3mode = altErr; // Alt out of range
                    }
            }
            else
            {
                mt3mode = azErr; // Az out of range
            }
            if (az2_c > 360) az2_c -= 360;
        }

        public object Clone()
        {
            return MemberwiseClone();
        }

        /// <summary>
        /// kalman 初期化ルーチン
        /// </summary>
        /// <param name="elem">読み出した要素</param> 
        public void kalman_init()
        {
            CvKalman kalman = kv_kalman;
            // 初期化(kalman)
            kalman_id = 0;
            Cv.SetIdentity(kalman.MeasurementMatrix, Cv.RealScalar(1.0));
            Cv.SetIdentity(kalman.ProcessNoiseCov, Cv.RealScalar(1e-4));
            Cv.SetIdentity(kalman.MeasurementNoiseCov, Cv.RealScalar(0.001));
            Cv.SetIdentity(kalman.ErrorCovPost, Cv.RealScalar(1.0));

            measurement.Zero();

            // 等速直線運動モデル(kalman)
            /* unsafe
            {
                kalman.DynamMatr[0] = 1.0f; kalman.DynamMatr[1] = 0.0f; kalman.DynamMatr[2] = 1.0f; kalman.DynamMatr[3] = 0.0f;
                kalman.DynamMatr[4] = 0.0f; kalman.DynamMatr[5] = 1.0f; kalman.DynamMatr[6] = 0.0f; kalman.DynamMatr[7] = 1.0f;
                kalman.DynamMatr[8] = 0.0f; kalman.DynamMatr[9] = 0.0f; kalman.DynamMatr[10] = 1.0f; kalman.DynamMatr[11] = 0.0f;
                kalman.DynamMatr[12] = 0.0f; kalman.DynamMatr[13] = 0.0f; kalman.DynamMatr[14] = 0.0f; kalman.DynamMatr[15] = 1.0f;
            }*/
            kalman.TransitionMatrix.Set2D(0, 0, 1.0f);
            kalman.TransitionMatrix.Set2D(0, 1, 0.0f);
            kalman.TransitionMatrix.Set2D(0, 2, 1.0f);
            kalman.TransitionMatrix.Set2D(0, 3, 0.0f);

            kalman.TransitionMatrix.Set2D(1, 0, 0.0f);
            kalman.TransitionMatrix.Set2D(1, 1, 1.0f);
            kalman.TransitionMatrix.Set2D(1, 2, 0.0f);
            kalman.TransitionMatrix.Set2D(1, 3, 1.0f);

            kalman.TransitionMatrix.Set2D(2, 0, 0.0f);
            kalman.TransitionMatrix.Set2D(2, 1, 0.0f);
            kalman.TransitionMatrix.Set2D(2, 2, 1.0f);
            kalman.TransitionMatrix.Set2D(2, 3, 0.0f);

            kalman.TransitionMatrix.Set2D(3, 0, 0.0f);
            kalman.TransitionMatrix.Set2D(3, 1, 0.0f);
            kalman.TransitionMatrix.Set2D(3, 2, 0.0f);
            kalman.TransitionMatrix.Set2D(3, 3, 1.0f);
        }
        /// <summary>
        /// kalman 初期化ルーチン
        /// </summary>
        /// <param name="elem">読み出した要素</param> 
        public void kalman_update()
        {
            if (kalman_id == 0)
            {
                kalman_init();
                // 初期値設定
                double errcov = 1.0; //仮
                kv_kalman.StatePost.Set1D(0, (float)az2_c);
                kv_kalman.StatePost.Set1D(1, (float)alt2_c);
                Cv.SetIdentity(kv_kalman.ErrorCovPost, Cv.RealScalar(errcov));
            }
            kalman_id++; 

            // 観測値(kalman)
            //float[] m = { (float)(az2_c), (float)(alt2_c) };
            //measurement = Cv.Mat(2, 1, MatrixType.F32C1, m);
            measurement.Set2D(0, 0, (float)az2_c);
            measurement.Set2D(1, 0, (float)alt2_c);

            // 観測誤差評価　FWHM=2.35σ
            double fwhm_az   = 0.005 * vaz2_kv / 2.0;
            double fwhm_alt  = 0.005 * valt2_kv / 2.0;
            double sigma_az  = (fwhm_az  / 2.35);
            double sigma_alt = (fwhm_alt / 2.35);
 
            kv_kalman.MeasurementNoiseCov.Set2D(0, 0, sigma_az * sigma_az);
            kv_kalman.MeasurementNoiseCov.Set2D(1, 1, sigma_alt * sigma_alt);
            //Cv.SetIdentity(kv_kalman.MeasurementNoiseCov, Cv.RealScalar(0.001));

            // 修正フェーズ(kalman)
            correction = Cv.KalmanCorrect(kv_kalman, measurement);
            // 予測フェーズ(kalman)
            prediction = Cv.KalmanPredict(kv_kalman);
            kvaz = prediction.DataArraySingle[0]; //ans
            kvalt = prediction.DataArraySingle[1]; //ans
            //kvx = prediction.DataArraySingle[2];
            //kvy = prediction.DataArraySingle[3];

        }

        /// <summary>
        /// CCD座標(cx,cy)->(az,alt)に変換
        /// </summary>
        //
        //   CCD座標(cx,cy):CCD中心からの誤差座標[pix]    Std. Cam が基準(cx = x-xc, cy = y-yc)
        //   中心位置(az_c,alt_c)と視野回転(theta_c)
        //   fl:焦点距離[mm],　ccdpx,ccdpy:ピクセル間隔[mm]

        public void cxcy2azalt(double cx, double cy,
               double az_c, double alt_c, int mode, double theta_c,
               double fl, double ccdpx, double ccdpy,
               ref double az, ref double alt)
        {
            double rad = Math.PI / 180.0;
            double cxmm, cymm;

            //ターゲットの方向余弦
            if (mode == mmEast)
            {
                cxmm = +cx * ccdpx; // -(+x)
                cymm = +cy * ccdpy; // -(-y)
            }
            else
            { //mmWest
                cxmm = -cx * ccdpx; // (+x)
                cymm = -cy * ccdpy; // (-y)
            }
            CvMat v1 = new CvMat(3, 1, MatrixType.F64C1);
            v1.Set2D(0, 0, fl);
            v1.Set2D(1, 0, cxmm);
            v1.Set2D(2, 0, cymm);
            v1.Normalize(v1);// 方向余弦化

            CvMat v2 = new CvMat(3, 1, MatrixType.F64C1);
            CvMat Rx = new CvMat(3, 3, MatrixType.F64C1);
            CvMat Rz = new CvMat(3, 3, MatrixType.F64C1);
            CvMat Ry = new CvMat(3, 3, MatrixType.F64C1);

            //Rx.rotX(-theta_c * rad); // 回転マトリクスをセット
            double sin = Math.Sin(-theta_c * rad);
            double cos = Math.Cos(-theta_c * rad);
            Rx.Set2D(0, 0, 1); Rx.Set2D(0, 1, 0); Rx.Set2D(0, 2, 0);
            Rx.Set2D(1, 0, 0); Rx.Set2D(1, 1, cos); Rx.Set2D(1, 2, -sin);
            Rx.Set2D(2, 0, 0); Rx.Set2D(2, 1, sin); Rx.Set2D(2, 2, cos);


            //Rz.rotZ(-az_c   *rad ); // 天球座標系と回転方向が逆なのでマイナス
            sin = Math.Sin(-az_c * rad);
            cos = Math.Cos(-az_c * rad);
            Rz.Set2D(0, 0, cos); Rz.Set2D(0, 1, -sin); Rz.Set2D(0, 2, 0);
            Rz.Set2D(1, 0, sin); Rz.Set2D(1, 1, cos); Rz.Set2D(1, 2, 0);
            Rz.Set2D(2, 0, 0); Rz.Set2D(2, 1, 0); Rz.Set2D(2, 2, 1);

            //Ry.rotY(-alt_c  *rad ); // 回転マトリクスをセット
            sin = Math.Sin(-alt_c * rad);
            cos = Math.Cos(-alt_c * rad);
            Ry.Set2D(0, 0, cos); Ry.Set2D(0, 1, 0); Ry.Set2D(0, 2, sin);
            Ry.Set2D(1, 0, 0); Ry.Set2D(1, 1, 1); Ry.Set2D(1, 2, 0);
            Ry.Set2D(2, 0, -sin); Ry.Set2D(2, 1, 0); Ry.Set2D(2, 2, cos);
            v2 = Rz * (Ry * (Rx * v1)); // 順番注意（画像中心をx軸に一致させている状態がスタート）

            // Retrun Val
            az = Math.Atan2(-v2.Get2D(1, 0), v2.Get2D(0, 0)) / rad;
            if (az < 0) az += 360;
            alt = Math.Asin(v2.Get2D(2, 0)) / rad;
        }
    }
}
