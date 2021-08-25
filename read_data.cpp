#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include <k4a/k4a.h>
#include <k4abt.h>
// #include <opencv2/opencv.hpp>

#include <chrono>
#include <ctime>

using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

/// <summary>
/// BOMを調べて、文字コードを判別する。
/// </summary>
/// <param name="bytes">文字コードを調べるデータ。</param>
/// <returns>BOMが見つかった時は、対応するEncodingオブジェクト。
/// 見つからなかった時は、null。</returns>

public static System.Text.Encoding DetectEncodingFromBOM(byte[] bytes)
{
    if (bytes.Length < 2)
    {
        return null;
    }
    if ((bytes[0] == 0xfe) && (bytes[1] == 0xff))
    {
        //UTF-16 BE
        return new System.Text.UnicodeEncoding(true, true);
    }
    if ((bytes[0] == 0xff) && (bytes[1] == 0xfe))
    {
        if ((4 <= bytes.Length) &&
            (bytes[2] == 0x00) && (bytes[3] == 0x00))
        {
            //UTF-32 LE
            return new System.Text.UTF32Encoding(false, true);
        }
        //UTF-16 LE
        return new System.Text.UnicodeEncoding(false, true);
    }
    if (bytes.Length < 3)
    {
        return null;
    }
    if ((bytes[0] == 0xef) && (bytes[1] == 0xbb) && (bytes[2] == 0xbf))
    {
        //UTF-8
        return new System.Text.UTF8Encoding(true, true);
    }
    if (bytes.Length < 4)
    {
        return null;
    }
    if ((bytes[0] == 0x00) && (bytes[1] == 0x00) &&
        (bytes[2] == 0xfe) && (bytes[3] == 0xff))
    {
        //UTF-32 BE
        return new System.Text.UTF32Encoding(true, true);
    }

    return null;
}

//Button1のClickイベントハンドラ
private void Button1_Click(object sender, System.EventArgs e)
{
    //テキストファイルを開く
    byte[] bs = System.IO.File.ReadAllBytes(TextBox1.Text);
    //.NET Framework 1.1以下では、次のようにする
    //System.IO.FileStream fs = new System.IO.FileStream(
    //    TextBox1.Text, System.IO.FileMode.Open,
    //    System.IO.FileAccess.Read);
    //byte[] bs = new byte[fs.Length];
    //fs.Read(bs, 0, bs.Length);
    //fs.Close();

    //文字コードを判別する
    System.Text.Encoding enc = DetectEncodingFromBOM(bs);

    if (enc == null)
    {
        Console.WriteLine("BOMが見つかりませんでした。");
    }

    //デコードして表示する
    //はじめのBOMを飛ばしてデコードする
    int bomLen = enc.GetPreamble().Length;
    RichTextBox1.Text = enc.GetString(bs, bomLen, bs.Length - bomLen);
}

int main()
{
    continue   
}