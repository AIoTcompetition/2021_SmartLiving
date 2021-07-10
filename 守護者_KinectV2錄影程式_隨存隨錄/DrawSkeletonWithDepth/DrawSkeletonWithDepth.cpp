/*
* IDE: Visual Studio 2019
* Language: C++
* Author: Kuihao Chang, 2021/07/10, https://github.com/kuihao
* Code modified from Heresy, 2015/02/25, https://kheresy.wordpress.com/2015/03/03/k4w-v2-part-7-user-skeleton/
* This code is used to load out information of BodyFrame (skeleton joints) and DepthFrame (ToF) to xml amd json file.
*/

/* [Kinect SDK 使用流程]
*  Step 1: Setting default Sensor
*		(a. 宣告 Senser pointer
*		(b. 由 Kinect API 更新 Senser pointer
*		(c. 由 Senser pointer 開啟 (open) sensor
*
*  Step 2: Getting frame source
*		(a. (依不同感測器個別) 宣告 Frame_source pointer (取得感測器資料位置) 與 Frame_reador pointer (讀出感測器資料)
*		(b. 由 Senser pointer 呼叫 API 更新 Frame_source pointer
*		(c. 由 Frame_source pointer 開啟 (open) Frame_reador pointer
*       (d. 由 Frame_reador pointer 呼叫 API 取得 frame data
*
*  Notice:
*		(深度影像處理
*			(a. 由 Frame_source pointer 透過 get_FrameDescription 可以取得當下影像的解析度資訊 (高、寬)
*/

// Standard Library
#include <iostream>
#include <typeinfo>
#include <string>
#include <direct.h>
#include <fstream>
#include <ctime>  


// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

// Json
#include <json.hpp>

// Namespace
using namespace std;
using namespace cv;
using json = nlohmann::json;

// 自訂輸出設定: Output operator for CameraSpacePoint
ostream& operator<<(ostream& rOS, const CameraSpacePoint& rPos)
{
	rOS << "(" << rPos.X << "/" << rPos.Y << "/" << rPos.Z << ")";
	return rOS;
}

// 自訂輸出設定: Output operator for Vector4
ostream& operator<<(ostream& rOS, const Vector4& rVec)
{
	rOS << "(" << rVec.x << "/" << rVec.y << "/" << rVec.z << "/" << rVec.w << ")";
	return rOS;
}

// 骨架節點編號次序 (MSDN可查表: https://docs.microsoft.com/en-us/previous-versions/windows/kinect/dn758663(v=ieb.10)?redirectedfrom=MSDN)
const int aJointIndex[20] = { 3, 20, 1, 0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 };

// 繪製骨架
void DrawLine(cv::Mat& rImg, const Joint& rJ1, const Joint& rJ2, ICoordinateMapper* pCMapper)
{
	if (rJ1.TrackingState == TrackingState_NotTracked || rJ2.TrackingState == TrackingState_NotTracked)
		return;

	DepthSpacePoint ptJ1, ptJ2;
	pCMapper->MapCameraPointToDepthSpace(rJ1.Position, &ptJ1);
	pCMapper->MapCameraPointToDepthSpace(rJ2.Position, &ptJ2);

	cv::line(rImg, cv::Point(ptJ1.X, ptJ1.Y), cv::Point(ptJ2.X, ptJ2.Y), cv::Vec3b(0, 0, 255), 5);
}

int main()
{
	// 0a. Control valiable: 流程控制 或 Debug之用 
	bool Debug_close_depth_record = false;
	int test = 0;
	// 判斷是否錄影
	bool flag_record = true;
	// 判斷是否自動儲存
	bool flag_auto_save = false;
	// 自動儲存時，判斷是否新開檔案
	bool flag_new_file = false;

	// 0b. 建立新資料夾 
	_mkdir("DepthFrames");
	_mkdir("SkeletonJoints");

	/* ======= 錄製影像前的準備 ====== */
		// 1a. Get default Sensor
	cout << "Try to get default sensor" << endl;
	// 1a.1. 宣告 感測器指標
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK)
	{
		cerr << "Get Sensor failed" << endl;
	}
	else
	{
		// 1b. Open sensor
		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
		}
		else
		{
			// 2a. Get frame source pointer
			cout << "Try to get source" << endl;
			// 2a.1. 宣告 深度資料來源指標
			IDepthFrameSource* pDepthFrameSource = nullptr;
			cout << "Try to get body source" << endl;
			// 2a.2. 宣告 身體資料來源指標
			IBodyFrameSource* pBodyFrameSource = nullptr;
			// 2b. 更新 資料來源指標
			if (pSensor->get_DepthFrameSource(&pDepthFrameSource) != S_OK)
			{
				cerr << "Can't get frame source" << endl;
			}
			else if (pSensor->get_BodyFrameSource(&pBodyFrameSource) != S_OK)
			{
				cerr << "Can't get body frame source" << endl;
			}
			else
			{
				// 2b.1. Get frame description
				// 2b.1.1. 宣告儲存寬高的變數 (只對 kinect 存取一次，以提升效能)
				int		iWidth = 0;
				int		iHeight = 0;
				// 2b.1.2. 宣告深度影像處理器指標
				IFrameDescription* pDepthFrameDescription = nullptr;
				// 2b.1.3. 取得目前 Kinect 深度鏡頭的畫面寬、高
				if (pDepthFrameSource->get_FrameDescription(&pDepthFrameDescription) == S_OK)
				{
					pDepthFrameDescription->get_Width(&iWidth);
					pDepthFrameDescription->get_Height(&iHeight);
					std::cout << "Got frame's width & hight: " << iWidth << " " << iHeight << std::endl; // iWidth:512 * iHeight:424
					pDepthFrameDescription->Release();
					pDepthFrameDescription = nullptr;
				}

				// 2b.2. Get the number of body
				// 2b.2.1. 取得目前 kinect 能捕捉到的最大人數
				INT32 iBodyCount = 0;
				if (pBodyFrameSource->get_BodyCount(&iBodyCount) != S_OK)
				{
					cerr << "Can't get body count" << endl;
					return -1;
				}
				cout << " > Can trace " << iBodyCount << " bodies" << endl;
				//  2b.2.2. 宣告/初始化 紀錄身體資料的陣列 
				IBody** aBody = new IBody * [iBodyCount];
				for (int i = 0; i < iBodyCount; i++)
					aBody[i] = nullptr;

				// 2b.3. get some dpeth only meta 取得深度距離建議數值
				UINT16 uDepthMin = 0, uDepthMax = 0;
				pDepthFrameSource->get_DepthMinReliableDistance(&uDepthMin);
				pDepthFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
				cout << "Reliable Distance: " << uDepthMin << " - " << uDepthMax << endl; // 建議深度 500 至 4,500

				// 2b.4. Perpare OpenCV
				// 2b.4.1. 用以儲存深度影像，Mat為 OpenCV 的資料結構
				cv::Mat mDepthImg(iHeight, iWidth, CV_16UC1);
				// 2b.4.2. 用以儲存轉換後的深度影像，用以輸出檢視畫面
				cv::Mat mImg8bit(iHeight, iWidth, CV_8UC1);
				// 2b.4.3. 預先開啟 OpenCV 檢視視窗
				//cv::namedWindow("Depth Map 8-bit");
				cv::namedWindow("Body Image");

				// * get CoordinateMapper
				ICoordinateMapper* pCoordinateMapper = nullptr;
				if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
				{
					cout << "Can't get coordinate mapper" << endl;
					return -1;
				}

				// 2c. Get frame reader
				cout << "Try to get frame reader" << endl;
				// 2c.1.1 宣告 深度影像讀取器指標
				IDepthFrameReader* pDepthFrameReader = nullptr;
				cout << "Try to get body frame reader" << endl;
				// 2c.1.2 宣告 身體讀取器指標
				IBodyFrameReader* pBodyFrameReader = nullptr;
				// 2c.2. 更新 讀取器指標
				if (pDepthFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
				{
					cerr << "Can't get frame reader" << endl;
				}
				else if (pBodyFrameSource->OpenReader(&pBodyFrameReader) != S_OK)
				{
					cerr << "Can't get body frame reader" << endl;
				}
				else
				{
					/* ======= 開始抓取影像 ====== */
										// Enter main loop
					cout << "***Enter main loop (開始抓取影像)***" << endl;
					cout << "***Press \'a\' key to record, ESC key to end the process.***" << endl;
					while (true)
					{
						// 設定 key 事件
						//char key = (char)cv::waitKey(30);
						//int key = (cv::waitKey(0) & 0xFF);

						/* ==== 狀態1: Idel-1 僅顯示目前畫面，未錄製影像 ==== */
						// 2d. Get last frame
						// 2d.1. 宣告 幀指標
						IDepthFrame* pDepthFrame = nullptr;
						IBodyFrame* pBodyFrame = nullptr;
						// 2d.2. 更新 幀指標
						if ((pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK) &&
							(pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK))
						{
							// 2d.3. 取幀資料並「複製」到 mDepthImg
							if ((pDepthFrame->CopyFrameDataToArray(iWidth * iHeight, reinterpret_cast<UINT16*>(mDepthImg.data)) == S_OK) &&
								(pBodyFrame->GetAndRefreshBodyData(iBodyCount, aBody) == S_OK))
							{
								// 2d.3.2. convert from 16bit to 8bit (顯示之用)
								mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);
								// 更新 深度影像的檢視畫面
								//cv::imshow("Depth Map 8-bit", mImg8bit);

								cv::Mat mImg = mImg8bit.clone();

								// * for each body
								for (int i = 0; i < iBodyCount; ++i)
								{
									IBody* pBody = aBody[i];

									// check if is tracked
									BOOLEAN bTracked = false;
									if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
									{
										// get joint position
										Joint aJoints[JointType::JointType_Count];
										if (pBody->GetJoints(JointType::JointType_Count, aJoints) == S_OK)
										{
											
											DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_SpineMid], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_SpineMid], aJoints[JointType_SpineShoulder], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_Neck], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_Neck], aJoints[JointType_Head], pCoordinateMapper);

											DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_ElbowLeft], aJoints[JointType_WristLeft], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_WristLeft], aJoints[JointType_HandLeft], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_HandTipLeft], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_ThumbLeft], pCoordinateMapper);

											DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_ElbowRight], aJoints[JointType_WristRight], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_WristRight], aJoints[JointType_HandRight], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_HandTipRight], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_ThumbRight], pCoordinateMapper);

											DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipLeft], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_HipLeft], aJoints[JointType_KneeLeft], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_KneeLeft], aJoints[JointType_AnkleLeft], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_AnkleLeft], aJoints[JointType_FootLeft], pCoordinateMapper);

											DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipRight], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_HipRight], aJoints[JointType_KneeRight], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_KneeRight], aJoints[JointType_AnkleRight], pCoordinateMapper);
											DrawLine(mImg, aJoints[JointType_AnkleRight], aJoints[JointType_FootRight], pCoordinateMapper);
										}
									}
								}
								// show image
								cv::imshow("Body Image", mImg);
							}
							else
							{
								// 若資料複製有錯誤
								cerr << "Data copy error" << endl;
							}

							// 2d. Release frame
							pDepthFrame->Release();
							pBodyFrame->Release();
						}

						// 3. 結束錄影並 Load out 資料至檔案 (注意: 要點選 openCV Window 再按按鍵才會觸發)
						if (cv::waitKey(10) == VK_ESCAPE)
						{
							// End main loop 結束抓取影像
							break;
						}

						/* ==== 輸入 d 開啟每 60 Frames 自動存檔功能 ==== */
						if (cv::waitKey(10) == 'd')
						{
							if (flag_auto_save)
							{
								cout << "***Auto-save Close!!***" << endl;
								flag_auto_save = false;
							}
							else
							{
								cout << "***Auto-save open!!***" << endl;
								flag_auto_save = true;
							}
						}

						/* ==== 狀態2: Record 錄製當前影像 ==== */
						if (cv::waitKey(10) == 'a')
						{
							/* ==== 60 Frame 自動存檔 ==== */
							if (flag_auto_save)
							{
								cout << "***Automatically save video data to file......***" << endl;
								cout << "Ready to record..." << endl;
								cout << "Press \'p\' key to stop/countinue record, \'s\' key to save to file." << endl;
								flag_new_file = true;

								// 自動建立新檔案
								while (flag_new_file)
								{
									// 取得系統時間
									char time_char[100];
									time_char[0] = '\0';
									time_t rawtime;
									//struct tm * timeinfo;
									struct tm timeinfo;
									time(&rawtime);
									//timeinfo = localtime(&rawtime);
									//localtime_s (timeinfo,&rawtime);
									localtime_s(&timeinfo, &rawtime);
									//strftime(str ,100 , "It is %Y-%m-%d %Z %X %x\n",timeinfo);
									strftime(time_char, 100, "%Y_%m_%d_%H_%M_%S", &timeinfo);
									string time_str = std::string(time_char);
									//cout << time_str << endl;

									// 宣告 INT變數 紀錄錄製到第幾幀
									int frame_count = 0;
									String frame_count_str = "";
									// Json 紀錄用: BodyFrame 序數
									int iTrackeBodyFrame_index_json = 0;
									String sTrackeBodyFrame_index_json = "";
									String sTrackeBody_index_json = "";
									// 宣告/開啟 XML 檔案，用以儲存深度影像資料
									String xml_path = "C:\\Users\\user\\Desktop\\Kincet_Data\\Depth\\Danger_";//Normal_
									xml_path.append(time_str);
									xml_path.append(".xml");
									cv::FileStorage CVfile(xml_path, cv::FileStorage::WRITE); //或存成 ext file 
									//cv::FileStorage CVfile("./DepthFrames/Test" + to_string(frame_count) + ".xml", cv::FileStorage::WRITE); //或存成 ext file
									// 宣告 MAT陣列 儲存幀資料
									std::vector<cv::Mat> video;
									// 宣告/開啟 json 檔案，儲存節點資料
									String json_path = "C:\\Users\\user\\Desktop\\Kincet_Data\\Skeleton\\Danger_";//Normal_
									json_path.append(time_str);
									json_path.append(".json");
									std::ofstream JSONfile(json_path); //time_str
									// 宣告 JSON 儲存節點資料
									json skeletons;

									while (true)
									{
										if (cv::waitKey(10) == 'p')
										{
											if (flag_record)
											{
												cout << "***Stop recording.***" << endl;
												flag_record = false; // defalut = true
												/* ==== 狀態3: Idel-2 僅顯示目前畫面，未錄影，但保有暫存資料 ==== */
											}
											else
											{
												cout << "***Countinue recording.***" << endl;
												flag_record = true;
												/* ==== 恢復至狀態2: Record ==== */
											}
										}

										// 2d. Get last frame
										// 2d.1. 宣告 幀指標
										IDepthFrame* pDepthFrame = nullptr;
										IBodyFrame* pBodyFrame = nullptr;
										// 2d.2. 更新 幀指標
										if ((pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK) &&
											(pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK))
										{
											// 2d.3. 取幀資料並「複製」到 mDepthImg
											if ((pDepthFrame->CopyFrameDataToArray(iWidth * iHeight, reinterpret_cast<UINT16*>(mDepthImg.data)) == S_OK) &&
												(pBodyFrame->GetAndRefreshBodyData(iBodyCount, aBody) == S_OK))
											{
												// 2d.3.1. convert from 16bit to 8bit (顯示之用)
												mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);
												// 更新 深度影像的檢視畫面
												cv::imshow("Depth Map 8-bit", mImg8bit);

												/* ==== 若 flag_record == true 則將影像存至陣列 ==== */
												if (flag_record)
												{
													frame_count++;
													frame_count_str = to_string(frame_count);

													// 2d.3.2. 將幀存入陣列
													video.push_back(mDepthImg);
													cout << "DepthFrame [" + frame_count_str + "] recorded." << endl;
													//cout << video.size() << endl;

													// * 紀錄確定有追蹤到的骨架數量
													int iTrackedBodyCount = 0;

													// * for each body 每個骨架依序輸出節點資料
													for (int i = 0; i < iBodyCount; i++)
													{
														IBody* pBody = aBody[i];

														// check if is tracked
														BOOLEAN bTracked = false;
														if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
														{
															++iTrackedBodyCount;
															// 有追蹤到1人時，才統計 BodyFrame
															if (iTrackedBodyCount & 1) { iTrackeBodyFrame_index_json++; }

															// 說明 User No.? 已被追蹤
															cout << "User " << i << " is under tracking" << endl;

															// Get joint position: 將所有節點資訊放入 aJoints 陣列
															Joint aJoints[JointType::JointType_Count];
															if (pBody->GetJoints(JointType::JointType_Count, aJoints) != S_OK)
															{
																cerr << "Get joints fail" << endl;
															}

															// Json 標籤記錄用
															sTrackeBodyFrame_index_json = to_string(iTrackeBodyFrame_index_json);
															sTrackeBody_index_json = to_string(iTrackedBodyCount);

															cout << "BodyFrame [" + sTrackeBodyFrame_index_json + "] recorded." << endl;

															// [!!!] Output information 依客製化需求取得不同位置的節點資料 (本作品選其中20個節點)
															for (int j = 0; j < 20; j++)
															{
																String sPosition = to_string(j);
																//JointType eJointType = JointType::JointType_HandRight;
																//const Joint& rJointPos = aJoints[eJointType];
																const Joint& rJointPos = aJoints[aJointIndex[j]]; //const 
																if (rJointPos.TrackingState == TrackingState_NotTracked)
																{
																	skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["x"] = 0;
																	skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["y"] = 0;
																	skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["z"] = 0;
																}
																else
																{
																	skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["x"] = rJointPos.Position.X;
																	skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["y"] = rJointPos.Position.Y;
																	skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["z"] = rJointPos.Position.Z;
																}
															}
														}
													}
												}
											}
											else
											{
												// 若資料複製有錯誤
												cerr << "Data copy error" << endl;
											}

											// 2d. Release frame
											pDepthFrame->Release();
											pBodyFrame->Release();
										}

										/* ==== 狀態4: Save 將暫存影像存至檔案 ==== */
										if ((frame_count >= 61) || (cv::waitKey(10) == 's'))
										{
											cout << "***Save video data to file......***" << endl;

											// 儲存深度影像
											// 寫入目前共錄製幾幀
											if (!Debug_close_depth_record) CVfile << "frame_counts" << frame_count;
											//CVfile << "frame_counts" << frame_count;
											// 將每一幀深度影像從 stack pop out
											if (!Debug_close_depth_record)
											{
												int temp_count;
												temp_count = frame_count;
												while (!video.empty())
												{
													CVfile << "frame_" + to_string(temp_count) << video.back();
													temp_count--;
													video.pop_back();
													cout << "(,,> w 0,,) writting into file..." << endl;
												}
											}

											// * 儲存骨架節點資料
											//cout << skeletons.dump(4) << endl;
											JSONfile << skeletons << std::endl;

											// 2b.4. 釋放 CVfile
											CVfile.release();
											JSONfile.close();
											// * 釋放暫存陣列
											video.clear();
											// * 釋放暫存陣列
											skeletons.clear();

											/* ==== 存檔後自動跳回XXX ==== */
											cout << "***File saved***" << endl;
											break;
										}

										// 停止自動存檔
										if (cv::waitKey(10) == VK_ESCAPE)
										{
											flag_new_file = false;
											break;
										}
									}// loop of recording
								}// loop of create a new file
								flag_auto_save = false;
								break;
							}// process of auto load out

							/* ==== 原始手動存檔 ==== */
							cout << "Ready to record..." << endl;
							cout << "Press \'p\' key to stop/countinue record, \'s\' key to save to file." << endl;

							// 取得系統時間
							char time_char[100];
							time_char[0] = '\0';
							time_t rawtime;
							//struct tm * timeinfo;
							struct tm timeinfo;
							time(&rawtime);
							//timeinfo = localtime(&rawtime);
							//localtime_s (timeinfo,&rawtime);
							localtime_s(&timeinfo, &rawtime);
							//strftime(str ,100 , "It is %Y-%m-%d %Z %X %x\n",timeinfo);
							strftime(time_char, 100, "%Y_%m_%d_%H_%M_%S", &timeinfo);
							string time_str = std::string(time_char);
							//cout << time_str << endl;

							// 宣告 INT變數 紀錄錄製到第幾幀
							int frame_count = 0;
							String frame_count_str = "";
							// Json 紀錄用: BodyFrame 序數
							int iTrackeBodyFrame_index_json = 0;
							String sTrackeBodyFrame_index_json = "";
							String sTrackeBody_index_json = "";
							// 宣告/開啟 XML 檔案，用以儲存深度影像資料
							String xml_path = "C:\\Users\\user\\Desktop\\Kincet_Data\\Depth\\Normal_";
							xml_path.append(time_str);
							xml_path.append(".xml");
							cv::FileStorage CVfile(xml_path, cv::FileStorage::WRITE); //或存成 ext file 
							//cv::FileStorage CVfile("./DepthFrames/Test" + to_string(frame_count) + ".xml", cv::FileStorage::WRITE); //或存成 ext file
							// 宣告 MAT陣列 儲存幀資料
							std::vector<cv::Mat> video;
							// 宣告/開啟 json 檔案，儲存節點資料
							String json_path = "C:\\Users\\user\\Desktop\\Kincet_Data\\Skeleton\\Normal_";
							json_path.append(time_str);
							json_path.append(".json");
							std::ofstream JSONfile(json_path); //time_str
							// 宣告 JSON 儲存節點資料
							json skeletons;

							while (true)
							{
								if (cv::waitKey(10) == 'p')
								{
									if (flag_record)
									{
										cout << "***Stop recording.***" << endl;
										flag_record = false; // defalut = true
										/* ==== 狀態3: Idel-2 僅顯示目前畫面，未錄影，但保有暫存資料 ==== */
									}
									else
									{
										cout << "***Countinue recording.***" << endl;
										flag_record = true;
										/* ==== 恢復至狀態2: Record ==== */
									}
								}

								// 2d. Get last frame
								// 2d.1. 宣告 幀指標
								IDepthFrame* pDepthFrame = nullptr;
								IBodyFrame* pBodyFrame = nullptr;
								// 2d.2. 更新 幀指標
								if ((pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK) &&
									(pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK))
								{
									// 2d.3. 取幀資料並「複製」到 mDepthImg
									if ((pDepthFrame->CopyFrameDataToArray(iWidth * iHeight, reinterpret_cast<UINT16*>(mDepthImg.data)) == S_OK) &&
										(pBodyFrame->GetAndRefreshBodyData(iBodyCount, aBody) == S_OK))
									{
										// 2d.3.1. convert from 16bit to 8bit (顯示之用)
										mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);
										// 更新 深度影像的檢視畫面
										cv::imshow("Depth Map 8-bit", mImg8bit);

										/* ==== 若 flag_record == true 則將影像存至陣列 ==== */
										if (flag_record)
										{
											frame_count++;
											frame_count_str = to_string(frame_count);

											// 2d.3.2. 將幀存入陣列
											video.push_back(mDepthImg);
											cout << "DepthFrame [" + frame_count_str + "] recorded." << endl;

											// * 紀錄確定有追蹤到的骨架數量
											int iTrackedBodyCount = 0;

											// * for each body 每個骨架依序輸出節點資料
											for (int i = 0; i < iBodyCount; i++)
											{
												IBody* pBody = aBody[i];

												// check if is tracked
												BOOLEAN bTracked = false;
												if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
												{
													++iTrackedBodyCount;
													// 有追蹤到1人時，才統計 BodyFrame
													if (iTrackedBodyCount & 1) { iTrackeBodyFrame_index_json++; }

													// 說明 User No.? 已被追蹤
													cout << "User " << i << " is under tracking" << endl;

													// Get joint position: 將所有節點資訊放入 aJoints 陣列
													Joint aJoints[JointType::JointType_Count];
													if (pBody->GetJoints(JointType::JointType_Count, aJoints) != S_OK)
													{
														cerr << "Get joints fail" << endl;
													}

													// Json 標籤記錄用
													sTrackeBodyFrame_index_json = to_string(iTrackeBodyFrame_index_json);
													sTrackeBody_index_json = to_string(iTrackedBodyCount);

													cout << "BodyFrame [" + sTrackeBodyFrame_index_json + "] recorded." << endl;

													// [!!!] Output information 依客製化需求取得不同位置的節點資料 (本作品選其中20個節點)
													for (int j = 0; j < 20; j++)
													{
														String sPosition = to_string(j);
														//JointType eJointType = JointType::JointType_HandRight;
														//const Joint& rJointPos = aJoints[eJointType];
														const Joint& rJointPos = aJoints[aJointIndex[j]]; //const 
														if (rJointPos.TrackingState == TrackingState_NotTracked)
														{
															skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["x"] = 0;
															skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["y"] = 0;
															skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["z"] = 0;
														}
														else
														{
															skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["x"] = rJointPos.Position.X;
															skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["y"] = rJointPos.Position.Y;
															skeletons[sTrackeBodyFrame_index_json][sTrackeBody_index_json][sPosition]["z"] = rJointPos.Position.Z;

															// 更嚴謹偵測
															/*
															if (rJointPos.TrackingState == TrackingState_Inferred){cout << "inferred " << endl;}
															else if (rJointPos.TrackingState == TrackingState_Tracked)
															{
																// 輸出節點三維空間資料
																cout << rJointPos.Position.X << "/"
																	<< rJointPos.Position.Y << "/"
																	<< rJointPos.Position.Z << endl;
															}*/
														}
													}
												}
											}

											// 輸出目前已追蹤的骨架數量
											/*
											if (iTrackedBodyCount > 0)
												cout << "Total " << iTrackedBodyCount << " bodies in this time\n" << endl;
											*/
										}
										//else
										//{
										//	cout << "***Idel... prass \'a\' key to record.***" << endl;
										//}
									}
									else
									{
										// 若資料複製有錯誤
										cerr << "Data copy error" << endl;
									}

									// 2d. Release frame
									pDepthFrame->Release();
									pBodyFrame->Release();
								}

								/* ==== 狀態4: Save 將暫存影像存至檔案 ==== */
								if (cv::waitKey(10) == 's')
								{
									cout << "***Save video data to file......***" << endl;

									// 儲存深度影像
									// 寫入目前共錄製幾幀
									if (!Debug_close_depth_record) CVfile << "frame_counts" << frame_count;
									//CVfile << "frame_counts" << frame_count;
									// 將每一幀深度影像從 stack pop out
									if (!Debug_close_depth_record)
									{
										int temp_count = frame_count;
										while (!video.empty())
										{
											CVfile << "frame_" + to_string(temp_count) << video.back();
											temp_count--;
											video.pop_back();
										}
									}
									/*
									int temp_count = frame_count;
									while (!video.empty())
									{
										CVfile << "frame_" + to_string(temp_count) << video.back();
										temp_count--;
										video.pop_back();
									}*/

									// * 儲存骨架節點資料
									//cout << skeletons.dump(4) << endl;
									JSONfile << skeletons << std::endl;

									// 2b.4. 釋放 CVfile
									CVfile.release();
									JSONfile.close();
									// * 釋放暫存陣列
									video.clear();
									// * 釋放暫存陣列
									skeletons.clear();

									/* ==== 存檔後自動跳回狀態1: Idel-1 ==== */
									cout << "***File saved***" << endl;
									cout << "***Press \'a\' key to record new video, ESC key to end the process.***" << endl;
									break;
								}
							}
						}// End of 狀態2

					}// End of main loop

					// 2c. release frame reader
					cout << "Release frame reader" << endl;
					pDepthFrameReader->Release();
					pDepthFrameReader = nullptr;
					pBodyFrameReader->Release();
					pBodyFrameReader = nullptr;

					// *
					delete[] aBody;
				}

				// 2a. Release Frame source
				cout << "Release frame source" << endl;
				pDepthFrameSource->Release();
				pDepthFrameSource = nullptr;
				pBodyFrameSource->Release();
				pBodyFrameSource = nullptr;
			}

			// 1b. Close Sensor
			cout << "close sensor" << endl;
			pSensor->Close();
		}

		// 1a.1. Release Sensor
		cout << "Release sensor" << endl;
		pSensor->Release();
		pSensor = nullptr;
	}

	return 0;
}
