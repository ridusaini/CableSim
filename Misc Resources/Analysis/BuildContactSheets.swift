import AppKit
import Foundation

private func fail(_ message: String) -> Never {
    FileHandle.standardError.write(Data(("error: \(message)\n").utf8))
    exit(1)
}

guard CommandLine.arguments.count == 3 else {
    fail("usage: BuildContactSheets <input-directory> <output-directory>")
}

let inputURL = URL(fileURLWithPath: CommandLine.arguments[1], isDirectory: true)
let outputURL = URL(fileURLWithPath: CommandLine.arguments[2], isDirectory: true)

let files: [URL]
do {
    files = try FileManager.default
        .contentsOfDirectory(at: inputURL, includingPropertiesForKeys: nil)
        .filter { ["jpg", "jpeg", "png"].contains($0.pathExtension.lowercased()) }
        .sorted { $0.lastPathComponent.localizedStandardCompare($1.lastPathComponent) == .orderedAscending }
    try FileManager.default.createDirectory(at: outputURL, withIntermediateDirectories: true)
} catch {
    fail("could not prepare contact sheets: \(error)")
}

let columns = 4
let rows = 4
let framesPerSheet = columns * rows
let imageWidth = 320
let imageHeight = 180
let labelHeight = 28
let cellHeight = imageHeight + labelHeight
let sheetSize = NSSize(width: columns * imageWidth, height: rows * cellHeight)

let labelStyle = NSMutableParagraphStyle()
labelStyle.alignment = .center
let labelAttributes: [NSAttributedString.Key: Any] = [
    .font: NSFont.monospacedSystemFont(ofSize: 14, weight: .medium),
    .foregroundColor: NSColor.white,
    .paragraphStyle: labelStyle,
]

for startIndex in stride(from: 0, to: files.count, by: framesPerSheet) {
    let sheet = NSImage(size: sheetSize)
    sheet.lockFocus()
    NSColor(calibratedWhite: 0.08, alpha: 1).setFill()
    NSRect(origin: .zero, size: sheetSize).fill()

    for localIndex in 0..<min(framesPerSheet, files.count - startIndex) {
        let row = localIndex / columns
        let column = localIndex % columns
        let frameURL = files[startIndex + localIndex]
        guard let frame = NSImage(contentsOf: frameURL) else {
            fail("could not load \(frameURL.path)")
        }

        let cellX = column * imageWidth
        let cellY = Int(sheetSize.height) - ((row + 1) * cellHeight)
        let imageRect = NSRect(
            x: cellX,
            y: cellY + labelHeight,
            width: imageWidth,
            height: imageHeight
        )
        frame.draw(in: imageRect, from: .zero, operation: .copy, fraction: 1)

        let labelRect = NSRect(
            x: cellX,
            y: cellY + 5,
            width: imageWidth,
            height: labelHeight - 5
        )
        frameURL.deletingPathExtension().lastPathComponent.draw(
            in: labelRect,
            withAttributes: labelAttributes
        )
    }

    sheet.unlockFocus()
    guard
        let tiff = sheet.tiffRepresentation,
        let bitmap = NSBitmapImageRep(data: tiff),
        let jpeg = bitmap.representation(using: .jpeg, properties: [.compressionFactor: 0.88])
    else {
        fail("could not encode contact sheet")
    }

    let sheetNumber = startIndex / framesPerSheet
    let filename = String(format: "contact_sheet_%02d.jpg", sheetNumber)
    do {
        try jpeg.write(to: outputURL.appendingPathComponent(filename))
    } catch {
        fail("could not write \(filename): \(error)")
    }
}

let sheetCount = Int(ceil(Double(files.count) / Double(framesPerSheet)))
print("Built \(sheetCount) contact sheets from \(files.count) frames.")
