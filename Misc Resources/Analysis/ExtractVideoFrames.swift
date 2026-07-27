import AppKit
import AVFoundation
import Foundation

private func fail(_ message: String) -> Never {
    FileHandle.standardError.write(Data(("error: \(message)\n").utf8))
    exit(1)
}

guard (4...6).contains(CommandLine.arguments.count) else {
    fail("usage: ExtractVideoFrames <input-video> <output-directory> <interval-seconds> [start-seconds] [end-seconds]")
}

let inputURL = URL(fileURLWithPath: CommandLine.arguments[1])
let outputURL = URL(fileURLWithPath: CommandLine.arguments[2], isDirectory: true)
guard let interval = Double(CommandLine.arguments[3]), interval > 0 else {
    fail("interval must be greater than zero")
}

let asset = AVURLAsset(url: inputURL)
let duration = CMTimeGetSeconds(asset.duration)
guard duration.isFinite, duration > 0 else {
    fail("could not read video duration")
}

let startTime = CommandLine.arguments.count >= 5
    ? (Double(CommandLine.arguments[4]) ?? 0)
    : 0
let endTime = CommandLine.arguments.count >= 6
    ? (Double(CommandLine.arguments[5]) ?? duration)
    : duration
guard startTime >= 0, startTime < duration, endTime > startTime else {
    fail("invalid start or end time")
}

do {
    try FileManager.default.createDirectory(
        at: outputURL,
        withIntermediateDirectories: true
    )
} catch {
    fail("could not create output directory: \(error)")
}

let generator = AVAssetImageGenerator(asset: asset)
generator.appliesPreferredTrackTransform = true
generator.maximumSize = CGSize(width: 1280, height: 720)
generator.requestedTimeToleranceBefore = CMTime(seconds: 0.05, preferredTimescale: 600)
generator.requestedTimeToleranceAfter = CMTime(seconds: 0.05, preferredTimescale: 600)

var timestamp = startTime
var frameIndex = 0
while timestamp < min(endTime, duration) {
    autoreleasepool {
        let requestedTime = CMTime(seconds: timestamp, preferredTimescale: 600)
        var actualTime = CMTime.zero

        do {
            let image = try generator.copyCGImage(at: requestedTime, actualTime: &actualTime)
            let bitmap = NSBitmapImageRep(cgImage: image)
            guard let jpeg = bitmap.representation(
                using: .jpeg,
                properties: [.compressionFactor: 0.82]
            ) else {
                fail("could not encode frame at \(timestamp) seconds")
            }

            let minutes = Int(timestamp) / 60
            let seconds = Int(timestamp) % 60
            let filename = String(
                format: "frame_%04d_%02d-%02d.jpg",
                frameIndex,
                minutes,
                seconds
            )
            try jpeg.write(to: outputURL.appendingPathComponent(filename))
        } catch {
            fail("could not extract frame at \(timestamp) seconds: \(error)")
        }
    }

    timestamp += interval
    frameIndex += 1
}

print(
    "Extracted \(frameIndex) frames from \(String(format: "%.3f", startTime)) " +
    "to \(String(format: "%.3f", min(endTime, duration))) seconds."
)
